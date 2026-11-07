# CR5A Web UI Guidelines

Read this before creating or changing React UI in `cr5a_web/`. Follow the repository's `AGENTS.md` first; this document adds the frontend-specific contracts.

## 1. Preserve the existing frontend stack

This project is currently:

- React 18 + TypeScript + Vite.
- React Router 6 with `BrowserRouter`, `Routes`, and `Route`.
- Tailwind CSS 3.
- Radix-based local UI primitives in `src/components/ui/`, styled in the shadcn/ui approach.
- Zustand for shared robot/application state.
- Three.js + React Three Fiber 8 + `urdf-loader` for the CR5 viewport.

Do not silently migrate React, React Router, Tailwind, Radix, Three.js, or the component system while adding a feature. In particular:

- Reuse the components in `src/components/ui/` before adding another UI dependency.
- New shared primitives belong in `src/components/ui/`; compose them into domain components rather than duplicating styles in pages.
- The current primitives use Radix. When using shadcn/ui examples or generated code, select APIs compatible with React 18, Tailwind 3, and Radix. Do not blindly run an installer or copy a Base UI / React 19 / Tailwind 4 example into this project.
- Use `cn()` from `src/lib/utils.ts` for conditional/merged Tailwind classes.
- Use `lucide-react` for icons already present in the dependency set. Do not add an icon package for one icon.

## 2. Visual hierarchy: operator first

The UI is an operator console, not a marketing dashboard. Optimize for quick interpretation and safe operation.

1. Keep the global navbar compact. It owns navigation and the single live robot-status badge.
2. Put the most-used and time-critical information first: robot availability, errors, and primary controls.
3. Make live actual values easy to scan. Use monospace/tabular numerals for coordinates, joint positions, IDs, and technical values.
4. Keep the 3D planning scene dominant on the Visualizer page. Keep target controls, plan result, and scene-object details close to it without competing with the viewport.
5. Put advanced/rarely used information behind disclosure or a secondary panel. Do not surface every ROS topic, parameter, field, or action on the main screen.
6. Avoid decorative cards, gradients, oversized headings, repeated badges, and unnecessary labels. Use whitespace and alignment rather than more containers.

Use the existing black/white/zinc palette as the base. Semantic status colors are reserved for operational meaning, not decoration.

## 3. One status vocabulary and one color mapping

Never define status colors independently in a page or component.

- Derive robot status through `src/lib/status.ts`.
- Render it with `src/components/status-badge.tsx`.
- Keep the CSS tokens in `src/index.css` aligned with the `RobotStatusKey` names.
- The navbar is the global source of the primary robot-status badge. Do not add a second copy of the same robot badge to every page header.

| State | Meaning | Color |
|---|---|---|
| `error` | Controller fault / active error | Red |
| `idle` | Connected, enabled, ready | Green |
| `paused` | Robot motion paused | Yellow / amber |
| `moving` | Running, jogging, executing | Blue |
| `planning` | MoveIt planning | Purple |
| `connecting` | WebSocket bridge connecting/reconnecting | Cyan |
| `initializing` | Controller initialization | Cyan |
| `disabled` | Connected but motors/robot disabled | Gray |
| `disconnected` | Robot or bridge disconnected | Gray |
| `unknown` | Not enough trustworthy telemetry | Gray |

Rules:

- Label every status; never communicate status through color alone.
- Do not display `idle` when state is missing or stale. Use `unknown` or `disconnected`.
- Error takes precedence over normal/active states. A failed operation is not the same thing as a robot controller fault; show operation result separately.
- Do not infer robot mode from a button click if live controller telemetry is available. A successful command acknowledgement is not proof that the controller changed state.
- Reuse `operationStatus()` for plan/execution badges instead of creating another palette.

## 4. Robot and motion interactions are safety-critical

Actual robot state and editable target state must remain visually and logically separate.

- Actual joint/TCP values are read-only telemetry. Target inputs/sliders are local drafts until the operator requests planning.
- Moving a slider or editing a TCP field must never publish a robot command or move the physical robot.
- Planning and execution are separate actions. A Plan action must not execute a trajectory.
- Disable Execute unless the bridge advertises execution capability, the robot is connected and enabled, and a successful plan exists.
- Clear or invalidate a stored plan when a target is edited or the actual robot start state changes. Do not offer execution of a plan computed from an old start state.
- Show the plan result, planning time, trajectory point count, and execution result without obscuring the live robot state.
- Use explicit units and frames next to values: TCP position in mm, RPY in degrees, joint positions in radians/degrees as labelled, and pose frame such as `dummy_link`.
- Confirm destructive scene actions (for example, clearing the planning scene) and any action that can initiate physical movement. Never relabel a normal Stop command as the safety-rated E-stop.
- Never weaken backend safety gates from frontend work. The bridge owns authorization and enforcement; a disabled UI control is not a safety boundary.
- Do not fabricate telemetry, joint ranges, collision-object positions, or TF transforms. Missing values must be displayed as unavailable. Do not draw a collision object in the wrong frame merely to make it visible.

## 5. Data flow: use the application WebSocket

The web console uses the CR5A bridge application protocol at `/ws/v1` on port `8000`. It does not use raw rosbridge at port `9090`.

- Use `useRobotSocket()` / `request()` from `src/lib/socket.tsx` and `src/lib/socket-client.ts` for robot snapshots, commands, planning, and scene requests.
- Subscribe to live channels through the existing socket client. Do not add one WebSocket per component.
- Read shared state from `src/store/robot-store.ts`; do not create parallel page-local copies of live telemetry.
- Put derived values in selectors/helpers or compute them from store state. Do not store redundant values that can be derived.
- Handle request errors, timeout, reconnect, missing telemetry, and bridge disconnection visibly. A pending request is not a successful command.
- Use the initial snapshot after connection and scene refresh after a scene-diff event. Do not poll high-frequency telemetry from each React component.
- Keep static mesh assets as HTTP assets under the existing `/assets/dobot_rviz/meshes/` path. The URDF/model metadata, robot state, MoveIt scene, and commands use the application WebSocket.

## 6. CR5 viewport rules

- Use the existing `RobotViewport` and its URDF loader before creating another renderer.
- Keep rendering components small and focused: robot model, collision geometry, trajectory, frames, grid, and camera controls should not become one large page component.
- CR5 joint positions from `/joint_states` are radians. Convert only at display/input boundaries and keep planning payloads in radians.
- Respect the URDF package/mesh prefix and the bridge-provided mesh base URL. Do not hardcode workstation paths.
- Scene geometry is meaningful only when its frame is known and transformed to the viewer frame. If TF is unavailable, show the frame mismatch rather than guessing.
- Dispose geometries, materials, blob URLs, event listeners, and timers when components unmount or assets reload.
- Provide a loading/error/unsupported-WebGL fallback; an empty canvas must not look like a valid empty planning scene.
- Keep the viewport responsive, but do not sacrifice its minimum useful size to fit secondary controls.

## 7. Component and accessibility conventions

- Keep route pages in `src/pages/`, reusable app/domain components in `src/components/`, shared UI primitives in `src/components/ui/`, state in `src/store/`, and shared logic/types in `src/lib/`.
- Split a component when it owns more than one independent responsibility or has become difficult to review. Do not split every markup fragment into a component.
- Use semantic `button`, `label`, `input`, `nav`, and heading elements. Every input needs a visible label and units where relevant.
- Icon-only controls need an accessible name (`aria-label`) and a tooltip/title where useful.
- Preserve visible keyboard focus. Tabs, dialogs, menus, and sliders must support keyboard interaction; prefer the existing Radix primitives for those patterns.
- Do not rely on color alone for errors, selection, robot state, or planning results. Include text/icon/shape cues.
- Respect reduced-motion preferences; keep animation subtle and never use animation as the only indication of a state change.
- Check keyboard use, narrow screens, and long/overflowing technical values before considering a UI change complete.

## 8. UI change checklist

Before committing a UI change:

- [ ] Read `AGENTS.md` and this document.
- [ ] Reuse the existing design tokens, status utility, socket client, and UI primitives.
- [ ] Verify loading, empty, error, disconnected, and disabled states.
- [ ] Verify actual values cannot be confused with editable targets.
- [ ] Verify planning cannot trigger execution and stale plans cannot be executed.
- [ ] Verify labels, units, frames, keyboard focus, and non-color status cues.
- [ ] Check desktop and narrow layouts.
- [ ] Run `pnpm build` from `cr5a_web/`.
- [ ] For bridge/API changes, also run Python syntax checks and the ROS 2 Humble package build/test in the project container.

## References

Use official documentation for APIs and interaction patterns. Check that examples match this repository's installed major versions before copying them.

- [React Quick Start](https://react.dev/learn) — components, JSX, event handling, and sharing state.
- [Thinking in React](https://react.dev/learn/thinking-in-react) — component hierarchy and minimal state.
- [React Router v6 concepts](https://reactrouter.com/docs/en/v6/start/concepts) — routes, layouts, and navigation.
- [shadcn/ui components](https://ui.shadcn.com/docs/components) — component patterns and available primitives.
- [shadcn/ui theming](https://ui.shadcn.com/docs/theming) — semantic color tokens and CSS-variable theming.
- [shadcn/ui manual installation](https://ui.shadcn.com/docs/installation/manual) — project integration reference; do not use it to migrate this existing stack.
- [Radix accessibility](https://www.radix-ui.com/primitives/docs/overview/accessibility) — keyboard navigation, ARIA semantics, and focus management.
- [Tailwind responsive design](https://tailwindcss.com/docs/responsive-design) — mobile-first responsive utilities.
- [Tailwind interaction states](https://tailwindcss.com/docs/hover-focus-and-other-states) — focus, disabled, and interaction styling.
- [WCAG 2.2 non-text contrast](https://www.w3.org/WAI/WCAG22/Understanding/non-text-contrast) — control and state-indicator contrast.
- [React Three Fiber introduction](https://r3f.docs.pmnd.rs/) and [Canvas API](https://r3f.docs.pmnd.rs/api/canvas) — 3D scene structure, render lifecycle, and fallback behavior.
