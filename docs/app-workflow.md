# SpaceFruit app flow and robot mapping

Verified in the signed-in browser on 2026-10-10 at http://localhost:5173/.
The owner authorized resetting the saved development garden. A local backup
was written to the app's ignored .local-backups directory before reset.

## Browser walkthrough completed

| Step | App action/result | C++ mission / physical job |
|---|---|---|
| 1 | Start garden check; state becomes Waiting For Confirmation | start -> map_plot -> read_environment; require valid measured bounds/readings |
| 2 | Recommended Lettuce, Radish, Carrot, Kale, Onion, one each; removed crops to leave Radish and Onion | App recommendations/editor supply the plan via set_plan; validate bounds, spacing, crop specs and count |
| 3 | Confirm planting plan; Waiting For Seeds | confirm_plan freezes the validated plan |
| 4 | Check seed tray; verified exactly one Radish and one Onion | load_seeds must match every requested crop/count, with no extras |
| 5 | Plant the garden; two profiles appear and planter count returns to zero | start_planting; navigate -> plant_seed for each cell; commit only after verified seed release |
| 6 | Check on my plants; visited 2/2 locations, weed progress 100%, two health alerts, zero weeds in this run | start_monitoring; navigate -> inspect_weeds -> remove_weeds if found -> inspect_health per planted cell |
| 7 | Open Radish profile -> Harvest plant; readiness check succeeded; Radish inventory becomes 1, cell becomes Empty Plot | start_harvest; navigate -> inspect_readiness -> harvest only when ready; verified collection increments inventory and empties cell |
| 8 | Open Empty Plot -> Plant something different -> select Carrot -> verify location -> Start planting | start_replant validates empty cell, crop spacing and one actually loaded seed; navigate -> plant_seed; create next profile generation |
| 9 | New Carrot profile replaces empty plot; Onion remains growing and harvested Radish remains in basket | Complete, with two growing cells and one harvested Radish |

Marketplace Trade, Buy/Sell, Donate and My listings were inspected separately.
They belong to the web app/database and do not map to motor or arm commands.
No marketplace transaction was performed.

## Branches verified by source and tests

The app's not-ready harvest branch leaves the plant in place and does not add
inventory. The C++ tests cover that branch, unknown readiness, failed/unverified
physical actions, incomplete weed removal, replayed results, exact seed counts,
out-of-bounds/overlapping plans, cancellation, timeout and safety stops.

The app uses simulated mapping, environment, camera models, planting and tools.
The browser run establishes its software sequence, not physical robot success.
The hardware version deliberately requires a real loaded seed for replanting;
the current app simulator loads it automatically.

## Hardware integration contract

A single Pi mission task owns Mission. Application requests invoke start,
set_plan, confirm_plan, load_seeds, start_planting, start_monitoring,
start_harvest or start_replant. The app handles crop recommendation/catalog and
its grid planning; pass the generated crop name/spacing/depth/positions to the
robot, which validates them before confirmation. No simulator results may be
forwarded as physical verification.

Dispatch pending() once by its unique action ID to a real device adapter. Pass
that ID back to complete_action after the device/model/operator verifies the
result. A serial ACK only establishes command reception. Job results must
establish arrival, seed release, weed removal, readiness or crop collection as
appropriate. Duplicate/late IDs cannot change profiles or inventory. Navigation
has a 120-second deadline; other jobs have 30 seconds. tick uses a monotonic clock
and real safety/localization state. Error/cancellation emits a stop job and stays
latched; do not blindly retry a tool that may have partly completed.

HardwareAction includes cell index, crop specification, target coordinates and
requested count. For navigate, the adapter must supply a reachable chassis work
pose using calibrated arm reach; plant coordinates are not automatically safe
wheel waypoints. Use Navigator only with actual camera pose and valid front/rear
ToF input. The listed TT motors have no encoders; PWM cannot establish travelled
metres. Turns require known swept clearance beyond front/rear range readings.

The RP2040 target supplies bounded motor/ToF control with CRC commands, command
expiry, arming and watchdog. It does not supply camera localization or gripper
kinematics. Those device/transport integrations remain unfinished. No physical
hardware bench verification has occurred.

The owner confirmed a harvesting gripper. Servo models, pulse calibration, arm
link geometry and grasp/collection verification are still missing. Seed and weed
tools have not been designed. Hardware adapters must return failed/unavailable
for unsupported jobs. The --simulate runtime is explicitly a test alternative;
it must never be enabled automatically for missing hardware.
