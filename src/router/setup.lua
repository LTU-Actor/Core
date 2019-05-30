R"=====(

_latitude = 0
_longitude = 0
_altitude = 0

function spin_once()
    if __spin_once() then
        error("reloading script")
    end
end

function spin_for(ms)
    if __spin_for(ms) then
        error("reloading script")
    end
end

-- https://stackoverflow.com/questions/365826/calculate-distance-between-2-gps-coordinates
function dist(lat, long)
    local d2r = math.pi / 180.0

    local dlong = (_longitude - long) * d2r
    local dlat = (_latitude - lat) * d2r
    local a = math.pow(math.sin(dlat/2.0), 2) + math.cos(lat*d2r) * math.cos(_latitude*d2r) * math.pow(math.sin(dlong/2.0), 2)
    local c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    return c * 6367000 -- meters
end

)====="
