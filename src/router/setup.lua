R"=====(

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

function dist(lat, long)
end

)====="
