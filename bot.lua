-- A Lua robot on the core binding supplied by the arena.
-- Precise contacts lead shots; remembered passive cues guide the search.
-- Clunkyard robot API 2.0.0: one core under every language, the same
-- answers everywhere. Facts (a call shows its arguments):
-- match: mode ruleset control robots is_3d tick_rate max_ticks
--   robot_id(slot)
-- clock: tick seconds remaining_ticks
-- arena: size gravity terrain_kind ground_height(x y)
-- me: health position velocity heading pitch roll angular_velocity
--   attitude grounded altitude applied
-- battery: charge capacity regen sustainable_drive compute
--   drive_cost(fraction)
-- drive: hover_lift
-- gun: aim_now cooldown_ticks spread max_power damage_scale
--   can_fire(power reserve) shot_speed(power) shot_cost(power)
--   reload_ticks(power) shot_damage(power) power_for_speed(speed)
--   power_for_flight_time(range seconds) lead(contact power)
--   lead_point(position velocity power)
-- scanner: last proximity_range reach(arc) arc_for_reach(range)
-- sensors: contacts nearest cues contact(slot) position_of(contact)
-- tracker: cue nearest_bearing_rate velocity(slot) position(slot)
--   last_seen(slot)
-- nav: zone zone.margin(point) zone.contains(point margin)
--   zone.distance_from(point) storm storm.margin(point)
--   storm.contains(point margin) storm.distance_from(point) hill
--   hill.margin(point) hill.contains(point margin)
--   hill.distance_from(point) pickups nearest_pickup(kind)
--   field_distance(a b)
-- score: me best armour
-- angles: normalize(degrees) angle_diff(target current)
--   bearing_to(from to) elevation_to(from to) direction(heading pitch)
--   distance(a b) clamp(value low high)
-- raw: number(path) text(path) has(path) flag(path) count(path) line()
-- Controls: drive.set(throttle turn pitch_turn)
--   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
--   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
--   tracker.configure(smoothing forget_ticks cue_ticks)
--   status.set(text value) raw.extra(json)
-- Every control argument is optional; a second call keeps what the first
-- set. Absent facts are your language's none. Names are snake_case here;
-- your language cases them its own way. Angles are degrees from the +x
-- axis; pitch is positive up.
-- Manual: https://clunkyard.com/manual#sdk
local cy = require("clunkyard")
local angles = cy.angles
local function steer(towards, current) return angles:clamp(angles:angle_diff(towards, current) / 45) end

-- The calibration kernel, xorshift-sqrt v1: work rounds of a 64-bit xorshift
-- and a square-root sum. Lua's >> is a logical shift.
local function calibration_work(work, tick)
  local x, sum = 88172645463325252 ~ tick, 0.0
  for _ = 1, work do
    x = x ~ (x << 13)
    x = x ~ (x >> 7)
    x = x ~ (x << 17)
    sum = sum + math.sqrt(x >> 11)
  end
  return (math.floor(sum) + (x >> 40)) % 1000000
end

local Starter = {}
function Starter:on_start() self.free = self.match.is_3d and self.arena.gravity.z == 0 end
function Starter:on_turn()
  local me, gun, scanner = self.me, self.gun, self.scanner
  self.drive:forces({brake=0.2})
  local target, cue = self.sensors.nearest, self.tracker.cue
  if target then
    local throttle = target.range > 350 and 0.7 or target.range < 200 and -0.4 or 0.2
    self.drive:set({throttle=throttle,turn=steer(target.bearing,me.heading),pitch_turn=steer(target.elevation,me.pitch)})
    scanner:set({heading=target.bearing,arc=20,pitch=target.elevation})
    local power = gun:power_for_flight_time(target.range,1)
    local aim = gun:lead(target,power)
    local heading,pitch = aim and aim.heading or target.bearing, aim and aim.pitch or target.elevation
    gun:aim({heading=heading,pitch=pitch})
    local now = gun.aim_now
    if math.abs(angles:angle_diff(heading,now.heading)) < 3 and math.abs(pitch-now.pitch) < 3 and gun:can_fire(power,10) then gun:fire({power=power}) end
    self.status:set({text="TRACK",value=target.range})
  elseif cue then
    local bearing,distance = angles:bearing_to(me.position,cue.position),angles:distance(me.position,cue.position)
    local elevation = self.free and angles:elevation_to(me.position,cue.position) or 0
    local error,tick = math.min(cue.bearing_error,30),self.clock.tick
    local across,up = (tick%5-2)/2*error,self.free and ((tick//5)%3-1)*error/2 or 0
    self.drive:set({throttle=0.7,turn=steer(bearing,me.heading),pitch_turn=steer(elevation,me.pitch)})
    scanner:set({heading=bearing+across,arc=scanner:arc_for_reach(distance+100),pitch=elevation+up})
    self.status:set({text="CUE",value=distance})
  else
    self.drive:set({throttle=0.5,turn=0.33,pitch_turn=steer(0,me.pitch)})
    scanner:set({heading=angles:normalize(scanner.last.heading+36),arc=40,pitch=self.free and ((self.clock.tick//10)%3-1)*35 or 0})
    self.status:set({text="SEARCH"})
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
  -- The arena's rate calibration, xorshift-sqrt v1; safe to delete.
  local work = self.raw:number("state.calibration.work")
  if work and work>0 and work%1==0 then self.status:set({text="WORK",value=calibration_work(math.tointeger(work),self.clock.tick)}) end
end
cy.run(Starter)
