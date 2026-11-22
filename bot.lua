-- Narrow the scan around a contact and lead shots from recent sightings.
-- Clunkyard guide: https://clunkyard.com/sdk/lua
local cy = require("clunkyard")
local angles = cy.angles
local function steer(towards, current) return angles:clamp(angles:angle_diff(towards, current) / 45) end

local SEARCH_ARC = 40  -- degrees: sweeps the circle in half a second
local HOLD_WIDTH = 60  -- units across at the target: it stays in the beam from tick to tick
local MIN_ARC = 6      -- degrees: the narrowest the beam holds
local WINDOW = 12      -- ticks of sightings the aim averages
local SIGHTINGS = 8    -- narrow sightings in the window before a shot
local LOOK_EVERY = 40  -- ticks between looks around the target, for a nearer robot
local REACQUIRE = 10   -- ticks to look again where a lost target was

local Starter = {}
function Starter:on_start()
  self.free = self.match.is_3d and self.arena.gravity.z == 0
  self.dodges, self.dodge_end, self.slot = 0, 0, 0
end

-- A straight line through the narrowest of the last WINDOW ticks' sightings:
-- where the robot is now, how fast it moves, and how many sightings it rests
-- on. Each sighting is off by its bearing_error; the line averages that out.
function Starter:fit(slot, tick)
  local recent, best = {}, math.huge
  for _, s in ipairs(self.tracker:history(slot)) do
    if s.tick > tick - WINDOW then recent[#recent+1] = s; best = math.min(best, s.bearing_error) end
  end
  if #recent < 2 then return nil end
  local seen = {}
  for _, s in ipairs(recent) do if s.bearing_error <= 1.5 * best then seen[#seen+1] = s end end
  local n = #seen
  if n < 2 then return nil end
  local t0, spread = 0, 0
  for _, s in ipairs(seen) do t0 = t0 + s.tick / n end
  for _, s in ipairs(seen) do spread = spread + (s.tick - t0) ^ 2 end
  local position, velocity = {}, {}
  for _, axis in ipairs({"x", "y", "z"}) do
    local mean, slope = 0, 0
    for _, s in ipairs(seen) do mean = mean + s.position[axis] / n end
    for _, s in ipairs(seen) do slope = slope + (s.tick - t0) * (s.position[axis] - mean) end
    slope = slope / spread
    position[axis], velocity[axis] = mean + slope * (tick - t0), slope * self.match.tick_rate
  end
  return position, velocity, n
end

function Starter:on_turn()
  local me, gun, scanner, tick = self.me, self.gun, self.scanner, self.clock.tick
  self.drive:forces({brake=0.2})
  local target, cue = self.sensors.nearest, self.tracker.cue
  local last_seen = self.slot ~= 0 and self.tracker:last_seen(self.slot) or nil
  if target then
    self.slot = target.slot
    local throttle = target.range > 350 and 0.7 or target.range < 200 and -0.4 or 0.2
    self.drive:set({throttle=throttle,turn=steer(target.bearing,me.heading),pitch_turn=steer(target.elevation,me.pitch)})
    -- Halve the beam each tick until it is HOLD_WIDTH across at the target:
    -- narrow enough for a precise bearing, wide enough to keep it in.
    local hold = math.max(MIN_ARC, math.deg(2 * math.atan(HOLD_WIDTH / 2, target.range)))
    -- In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks, widen
    -- it for a tick: a nearer robot it finds becomes the target.
    local arc = tick % LOOK_EVERY == 0 and SEARCH_ARC or math.max(hold, math.min(SEARCH_ARC, scanner.last.arc / 2))
    scanner:set({heading=target.bearing,arc=arc,pitch=target.elevation})
    local power = gun:power_for_flight_time(target.range,1)
    -- An exact contact: gun.lead is the intercept if the target holds course.
    -- A noisy one: lead the fitted line, and fire only once it rests on
    -- enough sightings.
    local aim, ready = nil, true
    if target.bearing_error > 0 then
      local position, velocity, count = self:fit(target.slot, tick)
      aim = position and gun:lead_point(position,velocity,power)
      ready = position ~= nil and count >= SIGHTINGS
    else
      aim = gun:lead(target,power)
    end
    local heading,pitch = aim and aim.heading or target.bearing, aim and aim.pitch or target.elevation
    gun:aim({heading=heading,pitch=pitch})
    local now = gun.aim_now
    if ready and math.abs(angles:angle_diff(heading,now.heading)) < 3 and math.abs(pitch-now.pitch) < 3 and gun:can_fire(power,10) then gun:fire({power=power}) end
    self.status:set({text=ready and "TRACK" or "NARROW",value=target.range})
  elseif last_seen and tick - last_seen <= REACQUIRE then
    -- Lost it: look again where it was last seen, widening the beam, before searching.
    local where = self.tracker:position(self.slot)
    local bearing = angles:bearing_to(me.position,where)
    local elevation = self.free and angles:elevation_to(me.position,where) or 0
    self.drive:set({throttle=0.5,turn=steer(bearing,me.heading),pitch_turn=steer(elevation,me.pitch)})
    scanner:set({heading=bearing,arc=math.min(SEARCH_ARC,scanner.last.arc*2),pitch=elevation})
    self.status:set({text="REACQUIRE"})
  elseif cue then
    local bearing,distance = angles:bearing_to(me.position,cue.position),angles:distance(me.position,cue.position)
    local elevation = self.free and angles:elevation_to(me.position,cue.position) or 0
    local error = math.min(cue.bearing_error,30)
    local across,up = (tick%5-2)/2*error,self.free and ((tick//5)%3-1)*error/2 or 0
    self.drive:set({throttle=0.7,turn=steer(bearing,me.heading),pitch_turn=steer(elevation,me.pitch)})
    scanner:set({heading=bearing+across,arc=scanner:arc_for_reach(distance+100),pitch=elevation+up})
    self.status:set({text="CUE",value=distance})
  else
    self.drive:set({throttle=0.5,turn=0.33,pitch_turn=steer(0,me.pitch)})
    scanner:set({heading=angles:normalize(scanner.last.heading+36),arc=SEARCH_ARC,pitch=self.free and ((tick//10)%3-1)*35 or 0})
    self.status:set({text="SEARCH"})
  end
  -- A simple dodge: vary it, or predict the shooter.
  local heard = false
  for _, c in ipairs(self.sensors.cues) do heard = heard or c.source == "muzzle" end
  if tick >= self.dodge_end and heard then
    self.dodges, self.dodge_end = self.dodges + 1, tick + 10
  end
  if tick < self.dodge_end then
    -- Off the line of fire: left, then right next time (turn is positive left, strafe right).
    local left = self.dodges % 2 == 1
    if self.match.control == "forces" then
      self.drive:set({throttle=0.5})
      self.drive:forces({strafe=left and -1 or 1})
    else
      self.drive:set({throttle=1,turn=left and 1 or -1})
    end
    self.status:set({text=left and "dodge:left" or "dodge:right",value=self.dodges})
  end
  if self.battery.charge < self.battery.capacity/4 then self.drive:set({throttle=self.drive:get("throttle")*0.5}) end
  local storm = self.nav.storm
  if storm and storm:margin(me.position)>-60 then
    local away = angles:bearing_to(storm.center,me.position)
    self.drive:set({throttle=0.8,turn=angles:clamp(angles:angle_diff(away,me.heading)/30)})
  end
  local zone = self.nav.zone
  if zone and zone:margin(me.position)<60 then
    local climb = zone.shape=="sphere" and angles:elevation_to(me.position,zone.center) or 0
    self.drive:set({throttle=0.8,turn=angles:clamp(angles:angle_diff(angles:bearing_to(me.position,zone.center),me.heading)/30),pitch_turn=angles:clamp((climb-me.pitch)/30)})
  end
end
cy.run(Starter)
