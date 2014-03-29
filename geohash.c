/*
 *Copyright (c) 2013-2014, yinqiwen <yinqiwen@gmail.com>
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of Redis nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 *BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stddef.h>
#include <stdio.h>
#include <math.h>

#include "lua.h"
#include "lauxlib.h"

#include "geohash.h"


/*
 * 0:success
 * -1:failed
 */
static int geohash_encode(GeoHashRange lat_range, GeoHashRange lon_range, double latitude, double longitude, uint8_t step, GeoHashBits* hash);
static int geohash_decode(GeoHashRange lat_range, GeoHashRange lon_range, GeoHashBits hash, GeoHashArea* area);

/*
 * Fast encode/decode version, more magic in implementation.
 */
static int geohash_fast_encode(GeoHashRange lat_range, GeoHashRange lon_range, double latitude, double longitude, uint8_t step, GeoHashBits* hash);
static int geohash_fast_decode(GeoHashRange lat_range, GeoHashRange lon_range, GeoHashBits hash, GeoHashArea* area);

static int geohash_get_neighbors(GeoHashBits hash, GeoHashNeighbors* neighbors);
static int geohash_get_neighbor(GeoHashBits hash, GeoDirection direction, GeoHashBits* neighbor);

static GeoHashBits geohash_next_leftbottom(GeoHashBits bits);
static GeoHashBits geohash_next_rightbottom(GeoHashBits bits);
static GeoHashBits geohash_next_lefttop(GeoHashBits bits);
static GeoHashBits geohash_next_righttop(GeoHashBits bits);

static int lua_geohash_encode(lua_State *L);
static int lua_geohash_fast_encode(lua_State *L);
static int lua_geohash_decode(lua_State *L);
static int lua_geohash_fast_decode(lua_State *L);
static int lua_geohash_get_neighbors(lua_State *L);
static int lua_geohash_get_neighbor(lua_State *L);
static int lua_geohash_next_leftbottom(lua_State *L);
static int lua_geohash_next_rightbottom(lua_State *L);
static int lua_geohash_next_lefttop(lua_State *L);
static int lua_geohash_next_righttop(lua_State *L);

static char *geohash_null = NULL;

static const struct luaL_Reg geohash_lib[] = {
    {"encode", lua_geohash_encode},
    {"fast_encode", lua_geohash_fast_encode},
    {"decode", lua_geohash_decode},
    {"fast_decode", lua_geohash_fast_decode},
    {"get_neighbors", lua_geohash_get_neighbors},
    {"get_neighbor", lua_geohash_get_neighbor},
    {"next_leftbottom", lua_geohash_next_leftbottom},
    {"next_rightbottom", lua_geohash_next_rightbottom},
    {"next_lefttop", lua_geohash_next_lefttop},
    {"next_righttop", lua_geohash_next_righttop},
    {NULL, NULL}
};

int luaopen_geohash(lua_State *L)
{
    luaL_register(L, "geohash", geohash_lib);

    lua_pushliteral(L, GEOHASH_INT_VERSION);
    lua_setfield(L, -2, "_VERSION");

    lua_pushlightuserdata(L, geohash_null);
    lua_setfield(L, -2, "null");

    return 1;
}

static int lua_geohash_encode(lua_State *L)
{
    GeoHashRange lat_range, lon_range;
    double latitude, longitude;
    uint8_t step;
    GeoHashBits hash;

    if (lua_gettop(L) != 7)
    {
        return luaL_error(L, "expected 7 arguments but got %d",
            lua_gettop(L));
    }

    lat_range.max = luaL_checknumber(L, 1);
    lat_range.min = luaL_checknumber(L, 2);
    lon_range.max = luaL_checknumber(L, 3);
    lon_range.min = luaL_checknumber(L, 4);
    latitude = luaL_checknumber(L, 5);
    longitude = luaL_checknumber(L, 6);
    step = luaL_checknumber(L, 7);

    if (step > 26 || step < 4)
    {
        lua_pushnil(L);
        lua_pushliteral(L, "step should be between 4 and 26");
        return 2;
    }

    geohash_encode(lat_range, lon_range, latitude, longitude, step, &hash);

    lua_newtable(L);

    lua_pushnumber(L, 1);
    lua_pushnumber(L, hash.bits);
    lua_settable(L, -3);

    lua_pushnumber(L, 2);
    lua_pushnumber(L, hash.step);
    lua_settable(L, -3);

    return 1;
}

static int lua_geohash_fast_encode(lua_State *L)
{
    GeoHashRange lat_range, lon_range;
    double latitude, longitude;
    uint8_t step;
    GeoHashBits hash;

    if (lua_gettop(L) != 7)
    {
        return luaL_error(L, "expected 7 arguments but got %d",
            lua_gettop(L));
    }

    lat_range.max = luaL_checknumber(L, 1);
    lat_range.min = luaL_checknumber(L, 2);
    lon_range.max = luaL_checknumber(L, 3);
    lon_range.min = luaL_checknumber(L, 4);
    latitude = luaL_checknumber(L, 5);
    longitude = luaL_checknumber(L, 6);
    step = luaL_checknumber(L, 7);

    if (step > 26 || step < 4)
    {
        lua_pushnil(L);
        lua_pushliteral(L, "step should be between 4 and 26");
        return 2;
    }

    geohash_fast_encode(lat_range, lon_range, latitude, longitude, step, &hash);

    lua_newtable(L);

    lua_pushnumber(L, 1);
    lua_pushnumber(L, hash.bits);
    lua_settable(L, -3);

    lua_pushnumber(L, 2);
    lua_pushnumber(L, hash.step);
    lua_settable(L, -3);

    return 1;
}

static int lua_geohash_decode(lua_State *L)
{
    GeoHashRange lat_range, lon_range;
    GeoHashBits hash;
    GeoHashArea area;

    if (lua_gettop(L) != 6)
    {
        return luaL_error(L, "expected 6 arguments but got %d",
            lua_gettop(L));
    }

    lat_range.max = luaL_checknumber(L, 1);
    lat_range.min = luaL_checknumber(L, 2);
    lon_range.max = luaL_checknumber(L, 3);
    lon_range.min = luaL_checknumber(L, 4);
    hash.bits = luaL_checknumber(L, 5);
    hash.step = luaL_checknumber(L, 6);

    if (hash.step > 26 || hash.step < 4)
    {
        lua_pushnil(L);
        lua_pushliteral(L, "step should be between 4 and 26");
        return 2;
    }

    geohash_decode(lat_range, lon_range, hash, &area);

    lua_newtable(L);

    lua_pushnumber(L, 1);
    lua_pushnumber(L, area.latitude.max);
    lua_settable(L, -3);

    lua_pushnumber(L, 2);
    lua_pushnumber(L, area.latitude.min);
    lua_settable(L, -3);

    lua_pushnumber(L, 3);
    lua_pushnumber(L, area.longitude.max);
    lua_settable(L, -3);

    lua_pushnumber(L, 4);
    lua_pushnumber(L, area.longitude.min);
    lua_settable(L, -3);

    return 1;
}

static int lua_geohash_fast_decode(lua_State *L)
{
    GeoHashRange lat_range, lon_range;
    GeoHashBits hash;
    GeoHashArea area;

    if (lua_gettop(L) != 6)
    {
        return luaL_error(L, "expected 6 arguments but got %d",
            lua_gettop(L));
    }

    lat_range.max = luaL_checknumber(L, 1);
    lat_range.min = luaL_checknumber(L, 2);
    lon_range.max = luaL_checknumber(L, 3);
    lon_range.min = luaL_checknumber(L, 4);
    hash.bits = luaL_checknumber(L, 5);
    hash.step = luaL_checknumber(L, 6);

    if (hash.step > 26 || hash.step < 4)
    {
        lua_pushnil(L);
        lua_pushliteral(L, "step should be between 4 and 26");
        return 2;
    }

    geohash_fast_decode(lat_range, lon_range, hash, &area);

    lua_newtable(L);

    lua_pushnumber(L, 1);
    lua_pushnumber(L, area.latitude.max);
    lua_settable(L, -3);

    lua_pushnumber(L, 2);
    lua_pushnumber(L, area.latitude.min);
    lua_settable(L, -3);

    lua_pushnumber(L, 3);
    lua_pushnumber(L, area.longitude.max);
    lua_settable(L, -3);

    lua_pushnumber(L, 4);
    lua_pushnumber(L, area.longitude.min);
    lua_settable(L, -3);

    return 1;
}

static int lua_geohash_get_neighbors(lua_State *L)
{
    GeoHashNeighbors neighbors;
    GeoHashBits hash;
    int i = 1;

    if (lua_gettop(L) != 2)
    {
        return luaL_error(L, "expected 2 arguments but got %d",
            lua_gettop(L));
    }

    hash.bits = luaL_checknumber(L, 1);
    hash.step = luaL_checknumber(L, 2);

    if (hash.step > 26 || hash.step < 4)
    {
        lua_pushnil(L);
        lua_pushliteral(L, "step should be between 4 and 26");
        return 2;
    }

    geohash_get_neighbors(hash, &neighbors);

    lua_newtable(L);

    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.north.bits);
    lua_settable(L, -3);
    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.north.step);
    lua_settable(L, -3);

    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.east.bits);
    lua_settable(L, -3);
    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.east.step);
    lua_settable(L, -3);

    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.west.bits);
    lua_settable(L, -3);
    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.west.step);
    lua_settable(L, -3);

    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.south.bits);
    lua_settable(L, -3);
    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.south.step);
    lua_settable(L, -3);

    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.north_east.bits);
    lua_settable(L, -3);
    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.north_east.step);
    lua_settable(L, -3);

    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.south_east.bits);
    lua_settable(L, -3);
    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.south_east.step);
    lua_settable(L, -3);

    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.north_west.bits);
    lua_settable(L, -3);
    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.north_west.step);
    lua_settable(L, -3);

    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.south_west.bits);
    lua_settable(L, -3);
    lua_pushnumber(L, i++);
    lua_pushnumber(L, neighbors.south_west.step);
    lua_settable(L, -3);

    return 1;
}

static int lua_geohash_get_neighbor(lua_State *L)
{
    GeoHashBits hash;
    GeoDirection direction;
    GeoHashBits neighbor;

    if (lua_gettop(L) != 3)
    {
        return luaL_error(L, "expected 3 arguments but got %d",
            lua_gettop(L));
    }

    hash.bits = luaL_checknumber(L, 1);
    hash.step = luaL_checknumber(L, 2);
    direction = luaL_checknumber(L, 3);

    if (hash.step > 26 || hash.step < 4)
    {
        lua_pushnil(L);
        lua_pushliteral(L, "step should be between 4 and 26");
        return 2;
    }

    if (direction > GEOHASH_NORT_EAST)
    {
        lua_pushnil(L);
        lua_pushliteral(L, "invalid direction");
        return 2;
    }

    geohash_get_neighbor(hash, direction, &neighbor);

    lua_newtable(L);

    lua_pushnumber(L, 1);
    lua_pushnumber(L, neighbor.bits);
    lua_settable(L, -3);

    lua_pushnumber(L, 2);
    lua_pushnumber(L, neighbor.step);
    lua_settable(L, -3);

    return 1;
}

static int lua_geohash_next_leftbottom(lua_State *L)
{
    GeoHashBits hash;

    if (lua_gettop(L) != 2)
    {
        return luaL_error(L, "expected 2 arguments but got %d",
            lua_gettop(L));
    }

    hash.bits = luaL_checknumber(L, 1);
    hash.step = luaL_checknumber(L, 2);

    if (hash.step > 25 || hash.step < 4)
    {
        lua_pushnil(L);
        lua_pushliteral(L, "step should be between 4 and 25");
        return 2;
    }

    hash = geohash_next_leftbottom(hash);

    lua_newtable(L);

    lua_pushnumber(L, 1);
    lua_pushnumber(L, hash.bits);
    lua_settable(L, -3);

    lua_pushnumber(L, 2);
    lua_pushnumber(L, hash.step);
    lua_settable(L, -3);

    return 1;
}

static int lua_geohash_next_rightbottom(lua_State *L)
{
    GeoHashBits hash;

    if (lua_gettop(L) != 2)
    {
        return luaL_error(L, "expected 2 arguments but got %d",
            lua_gettop(L));
    }

    hash.bits = luaL_checknumber(L, 1);
    hash.step = luaL_checknumber(L, 2);

    if (hash.step > 25 || hash.step < 4)
    {
        lua_pushnil(L);
        lua_pushliteral(L, "step should be between 4 and 25");
        return 2;
    }

    hash = geohash_next_rightbottom(hash);

    lua_newtable(L);

    lua_pushnumber(L, 1);
    lua_pushnumber(L, hash.bits);
    lua_settable(L, -3);

    lua_pushnumber(L, 2);
    lua_pushnumber(L, hash.step);
    lua_settable(L, -3);

    return 1;
}

static int lua_geohash_next_lefttop(lua_State *L)
{
    GeoHashBits hash;

    if (lua_gettop(L) != 2)
    {
        return luaL_error(L, "expected 2 arguments but got %d",
            lua_gettop(L));
    }

    hash.bits = luaL_checknumber(L, 1);
    hash.step = luaL_checknumber(L, 2);

    if (hash.step > 25 || hash.step < 4)
    {
        lua_pushnil(L);
        lua_pushliteral(L, "step should be between 4 and 25");
        return 2;
    }

    hash = geohash_next_lefttop(hash);

    lua_newtable(L);

    lua_pushnumber(L, 1);
    lua_pushnumber(L, hash.bits);
    lua_settable(L, -3);

    lua_pushnumber(L, 2);
    lua_pushnumber(L, hash.step);
    lua_settable(L, -3);

    return 1;
}

static int lua_geohash_next_righttop(lua_State *L)
{
    GeoHashBits hash;

    if (lua_gettop(L) != 2)
    {
        return luaL_error(L, "expected 2 arguments but got %d",
            lua_gettop(L));
    }

    hash.bits = luaL_checknumber(L, 1);
    hash.step = luaL_checknumber(L, 2);

    if (hash.step > 25 || hash.step < 4)
    {
        lua_pushnil(L);
        lua_pushliteral(L, "step should be between 4 and 25");
        return 2;
    }

    hash = geohash_next_righttop(hash);

    lua_newtable(L);

    lua_pushnumber(L, 1);
    lua_pushnumber(L, hash.bits);
    lua_settable(L, -3);

    lua_pushnumber(L, 2);
    lua_pushnumber(L, hash.step);
    lua_settable(L, -3);

    return 1;
}

/**
 * Hashing works like this:
 * Divide the world into 4 buckets.  Label each one as such:
 *  -----------------
 *  |       |       |
 *  |       |       |
 *  | 0,1   | 1,1   |
 *  -----------------
 *  |       |       |
 *  |       |       |
 *  | 0,0   | 1,0   |
 *  -----------------
 */

static int geohash_encode(
        GeoHashRange lat_range, GeoHashRange lon_range,
        double latitude, double longitude, uint8_t step, GeoHashBits* hash)
{
    if (NULL == hash || step > 32 || step == 0)
    {
        return -1;
    }
    hash->bits = 0;
    hash->step = step;
    uint8_t i = 0;
    if (latitude < lat_range.min || latitude > lat_range.max
     || longitude < lon_range.min || longitude > lon_range.max)
    {
        return -1;
    }

    for (; i < step; i++)
    {
        uint8_t lat_bit, lon_bit;
        if (lat_range.max - latitude >= latitude - lat_range.min)
        {
            lat_bit = 0;
            lat_range.max = (lat_range.max + lat_range.min) / 2;
        }
        else
        {
            lat_bit = 1;
            lat_range.min = (lat_range.max + lat_range.min) / 2;
        }
        if (lon_range.max - longitude >= longitude - lon_range.min)
        {
            lon_bit = 0;
            lon_range.max = (lon_range.max + lon_range.min) / 2;
        }
        else
        {
            lon_bit = 1;
            lon_range.min = (lon_range.max + lon_range.min) / 2;
        }
        hash->bits <<= 1;
        hash->bits += lon_bit;
        hash->bits <<= 1;
        hash->bits += lat_bit;
    }
    return 0;
}

static inline uint8_t get_bit(uint64_t bits, uint8_t pos)
{
    return (bits >> pos) & 0x01;
}

static int geohash_decode(
        GeoHashRange lat_range, GeoHashRange lon_range, GeoHashBits hash, GeoHashArea* area)
{
    if (NULL == area)
    {
        return -1;
    }
    area->hash = hash;
    uint8_t i = 0;
    area->latitude.min = lat_range.min;
    area->latitude.max = lat_range.max;
    area->longitude.min = lon_range.min;
    area->longitude.max = lon_range.max;
    for (; i < hash.step; i++)
    {
        uint8_t lat_bit, lon_bit;
        lon_bit = get_bit(hash.bits, (hash.step - i) * 2 - 1);
        lat_bit = get_bit(hash.bits, (hash.step - i) * 2 - 2);
        if (lat_bit == 0)
        {
            area->latitude.max = (area->latitude.max + area->latitude.min) / 2;
        }
        else
        {
            area->latitude.min = (area->latitude.max + area->latitude.min) / 2;
        }
        if (lon_bit == 0)
        {
            area->longitude.max = (area->longitude.max + area->longitude.min) / 2;
        }
        else
        {
            area->longitude.min = (area->longitude.max + area->longitude.min) / 2;
        }
    }
    return 0;
}

static inline uint64_t interleave64(uint32_t xlo, uint32_t ylo)
{
    static const uint64_t B[] =
        { 0x5555555555555555, 0x3333333333333333, 0x0F0F0F0F0F0F0F0F, 0x00FF00FF00FF00FF, 0x0000FFFF0000FFFF };
    static const unsigned int S[] =
        { 1, 2, 4, 8, 16 };

    uint64_t x = xlo; // Interleave lower  bits of x and y, so the bits of x
    uint64_t y = ylo; // are in the even positions and bits from y in the odd; //https://graphics.stanford.edu/~seander/bithacks.html#InterleaveBMN

    // x and y must initially be less than 2**32.

    x = (x | (x << S[4])) & B[4];
    y = (y | (y << S[4])) & B[4];

    x = (x | (x << S[3])) & B[3];
    y = (y | (y << S[3])) & B[3];

    x = (x | (x << S[2])) & B[2];
    y = (y | (y << S[2])) & B[2];

    x = (x | (x << S[1])) & B[1];
    y = (y | (y << S[1])) & B[1];

    x = (x | (x << S[0])) & B[0];
    y = (y | (y << S[0])) & B[0];

    return x | (y << 1);
}

static inline uint64_t deinterleave64(uint64_t interleaved)
{
    static const uint64_t B[] =
        { 0x5555555555555555, 0x3333333333333333, 0x0F0F0F0F0F0F0F0F, 0x00FF00FF00FF00FF, 0x0000FFFF0000FFFF,
                0x00000000FFFFFFFF };
    static const unsigned int S[] =
        { 0, 1, 2, 4, 8, 16 };

    uint64_t x = interleaved; ///reverse the interleave process (http://stackoverflow.com/questions/4909263/how-to-efficiently-de-interleave-bits-inverse-morton)
    uint64_t y = interleaved >> 1;

    x = (x | (x >> S[0])) & B[0];
    y = (y | (y >> S[0])) & B[0];

    x = (x | (x >> S[1])) & B[1];
    y = (y | (y >> S[1])) & B[1];

    x = (x | (x >> S[2])) & B[2];
    y = (y | (y >> S[2])) & B[2];

    x = (x | (x >> S[3])) & B[3];
    y = (y | (y >> S[3])) & B[3];

    x = (x | (x >> S[4])) & B[4];
    y = (y | (y >> S[4])) & B[4];

    x = (x | (x >> S[5])) & B[5];
    y = (y | (y >> S[5])) & B[5];

    return x | (y << 32);
}

static int geohash_fast_encode(
        GeoHashRange lat_range, GeoHashRange lon_range, double latitude,
        double longitude, uint8_t step,  GeoHashBits* hash)
{
    if (NULL == hash || step > 32 || step == 0)
    {
        return -1;
    }
    hash->bits = 0;
    hash->step = step;
    if   (latitude < lat_range.min || latitude > lat_range.max
      || longitude < lon_range.min || longitude > lon_range.max)
    {
        return -1;
    }

    // The algorithm computes the morton code for the geohash location within
    // the range this can be done MUCH more efficiently using the following code

    //compute the coordinate in the range 0-1
    double lat_offset = (latitude - lat_range.min) / (lat_range.max - lat_range.min);
    double lon_offset = (longitude - lon_range.min) / (lon_range.max - lon_range.min);

    //convert it to fixed point based on the step size
    lat_offset *= (1LL << step);
    lon_offset *= (1LL << step);

    uint32_t ilato = (uint32_t) lat_offset;
    uint32_t ilono = (uint32_t) lon_offset;

    //interleave the bits to create the morton code.  No branching and no bounding
    hash->bits = interleave64(ilato, ilono);
    return 0;
}

static int geohash_fast_decode(GeoHashRange lat_range, GeoHashRange lon_range, GeoHashBits hash, GeoHashArea* area)
{
    if (NULL == area)
    {
        return -1;
    }
    area->hash = hash;
    uint8_t step = hash.step;
    uint64_t xyhilo = deinterleave64(hash.bits); //decode morton code

    double lat_scale = lat_range.max - lat_range.min;
    double lon_scale = lon_range.max - lon_range.min;

    uint32_t ilato = xyhilo;        //get back the original integer coordinates
    uint32_t ilono = xyhilo >> 32;

    //double lat_offset=ilato;
    //double lon_offset=ilono;
    //lat_offset /= (1<<step);
    //lon_offset /= (1<<step);

    //the ldexp call converts the integer to a double,then divides by 2**step to get the 0-1 coordinate, which is then multiplied times scale and added to the min to get the absolute coordinate
//    area->latitude.min = lat_range.min + ldexp(ilato, -step) * lat_scale;
//    area->latitude.max = lat_range.min + ldexp(ilato + 1, -step) * lat_scale;
//    area->longitude.min = lon_range.min + ldexp(ilono, -step) * lon_scale;
//    area->longitude.max = lon_range.min + ldexp(ilono + 1, -step) * lon_scale;

    /*
     * much faster than 'ldexp'
     */
    area->latitude.min = lat_range.min + (ilato * 1.0 / (1ull << step)) * lat_scale;
    area->latitude.max = lat_range.min + ((ilato + 1) * 1.0 / (1ull << step)) * lat_scale;
    area->longitude.min = lon_range.min + (ilono * 1.0 / (1ull << step)) * lon_scale;
    area->longitude.max = lon_range.min + ((ilono + 1) * 1.0 / (1ull << step)) * lon_scale;

    return 0;
}

static int geohash_move_x(GeoHashBits* hash, int8_t d)
{
    if (d == 0)
        return 0;
    uint64_t x = hash->bits & 0xaaaaaaaaaaaaaaaaLL;
    uint64_t y = hash->bits & 0x5555555555555555LL;

    uint64_t zz = 0x5555555555555555LL >> (64 - hash->step * 2);
    if (d > 0)
    {
        x = x + (zz + 1);
    }
    else
    {
        x = x | zz;
        x = x - (zz + 1);
    }
    x &= (0xaaaaaaaaaaaaaaaaLL >> (64 - hash->step * 2));
    hash->bits = (x | y);
    return 0;
}

static int geohash_move_y(GeoHashBits* hash, int8_t d)
{
    if (d == 0)
        return 0;
    uint64_t x = hash->bits & 0xaaaaaaaaaaaaaaaaLL;
    uint64_t y = hash->bits & 0x5555555555555555LL;

    uint64_t zz = 0xaaaaaaaaaaaaaaaaLL >> (64 - hash->step * 2);
    if (d > 0)
    {
        y = y + (zz + 1);
    }
    else
    {
        y = y | zz;
        y = y - (zz + 1);
    }
    y &= (0x5555555555555555LL >> (64 - hash->step * 2));
    hash->bits = (x | y);
    return 0;
}

static int geohash_get_neighbors(GeoHashBits hash, GeoHashNeighbors* neighbors)
{
    geohash_get_neighbor(hash, GEOHASH_NORTH, &neighbors->north);
    geohash_get_neighbor(hash, GEOHASH_EAST, &neighbors->east);
    geohash_get_neighbor(hash, GEOHASH_WEST, &neighbors->west);
    geohash_get_neighbor(hash, GEOHASH_SOUTH, &neighbors->south);
    geohash_get_neighbor(hash, GEOHASH_SOUTH_WEST, &neighbors->south_west);
    geohash_get_neighbor(hash, GEOHASH_SOUTH_EAST, &neighbors->south_east);
    geohash_get_neighbor(hash, GEOHASH_NORT_WEST, &neighbors->north_west);
    geohash_get_neighbor(hash, GEOHASH_NORT_EAST, &neighbors->north_east);
    return 0;
}

static int geohash_get_neighbor(GeoHashBits hash, GeoDirection direction, GeoHashBits* neighbor)
{
    if (NULL == neighbor)
    {
        return -1;
    }
    *neighbor = hash;
    switch (direction)
    {
        case GEOHASH_NORTH:
        {
            geohash_move_x(neighbor, 0);
            geohash_move_y(neighbor, 1);
            break;
        }
        case GEOHASH_SOUTH:
        {
            geohash_move_x(neighbor, 0);
            geohash_move_y(neighbor, -1);
            break;
        }
        case GEOHASH_EAST:
        {
            geohash_move_x(neighbor, 1);
            geohash_move_y(neighbor, 0);
            break;
        }
        case GEOHASH_WEST:
        {
            geohash_move_x(neighbor, -1);
            geohash_move_y(neighbor, 0);
            break;
        }
        case GEOHASH_SOUTH_WEST:
        {
            geohash_move_x(neighbor, -1);
            geohash_move_y(neighbor, -1);
            break;
        }
        case GEOHASH_SOUTH_EAST:
        {
            geohash_move_x(neighbor, 1);
            geohash_move_y(neighbor, -1);
            break;
        }
        case GEOHASH_NORT_WEST:
        {
            geohash_move_x(neighbor, -1);
            geohash_move_y(neighbor, 1);
            break;
        }
        case GEOHASH_NORT_EAST:
        {
            geohash_move_x(neighbor, 1);
            geohash_move_y(neighbor, 1);
            break;
        }
        default:
        {
            return -1;
        }
    }
    return 0;
}

static GeoHashBits geohash_next_leftbottom(GeoHashBits bits)
{
    GeoHashBits newbits = bits;
    newbits.step++;
    newbits.bits <<= 2;
    return newbits;
}
static GeoHashBits geohash_next_rightbottom(GeoHashBits bits)
{
    GeoHashBits newbits = bits;
    newbits.step++;
    newbits.bits <<= 2;
    newbits.bits += 2;
    return newbits;
}
static GeoHashBits geohash_next_lefttop(GeoHashBits bits)
{
    GeoHashBits newbits = bits;
    newbits.step++;
    newbits.bits <<= 2;
    newbits.bits += 1;
    return newbits;
}

static GeoHashBits geohash_next_righttop(GeoHashBits bits)
{
    GeoHashBits newbits = bits;
    newbits.step++;
    newbits.bits <<= 2;
    newbits.bits += 3;
    return newbits;
}
