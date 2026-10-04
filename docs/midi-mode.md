# MIDI controller mode

## Goal

Add a USB MIDI mode for playing scale notes and chords on the existing Touchbox buttons. The controller should be playable without using the touchscreen. The screen may display the current settings, but touch gestures and touch position are not MIDI controls in this design.

The existing HID modes remain available. When the mode selection menu appears, press MCP1 A7 (the tonic-select menu button) to choose MIDI mode. There is no need to hold it while powering on.

## Agreed playing controls

| Physical controls | MIDI mode behavior |
| --- | --- |
| X, Y, A, B, RT, LT, RB, LB | Scale degrees 1–8 in that order, including the octave of the first note. Holding more than one plays multiple notes. |
| Four left-hand buttons (SSBM L, left, down, right) | Held chord qualities: major, minor, diminished, augmented, in that order. With none held, a note button plays a single note. |
| Two left-thumb buttons | Held shifts of -1 and +1 semitone, in that order. |
| Remaining left-thumb button | Held shift of +1 octave. The base octave is selected separately. |
| Five right-thumb buttons (SSBM right stick middle, right, up, left, down) | Held additions of flat 7, major 7, 9, 11, and 13, in that order. |
| Dedicated physical drone button | Tap to toggle a low drone note. Hold it and press a note button to select the drone pitch. The note button is used for selection and does not also play a note. |

Button switches provide no usable playing velocity measurement, so note-on messages use a fixed velocity of 100.

### Drone interaction

The initial drone pitch is the tonic in a low register. A tap toggles it on or off. Holding the drone button while pressing one of the eight note buttons selects a new pitch from the current scale; the semitone shift buttons can reach pitches outside that scale. If the drone is sounding, selecting a pitch should end the old drone note and start the new one. The button's tap action must occur on release so a hold-and-select gesture does not accidentally toggle the drone first.

The drone is latched: releasing its button does not stop the note. Its sounding state and selected pitch appear on the screen. The default drone is the tonic one octave below the base note and follows changes to the tonic or base octave. A custom selected drone remains at its selected absolute MIDI pitch when those settings change.

## Configuration controls

The six menu buttons change settings between phrases:

| Menu button | Action |
| --- | --- |
| MCP1 A7: tonic select | Hold and press one of the note buttons to set the tonic to that button's currently mapped pitch class. |
| MCP1 B0/B1: scale previous/next | Cycle through major, natural minor, Dorian, and Mixolydian. |
| MCP2 A5/A6: base octave down/up | Set the unshifted playing octave between C2 and C5. |
| MCP2 A7: drone | Tap to toggle; hold with a note button to select its pitch, as described above. |

When selecting a tonic or drone pitch, note buttons are selectors and do not emit ordinary note-on messages.
MIDI mode starts in C major with C4 as the first note and C3 as the default drone.

## Chord behavior

Use the chosen note as the root. The four quality buttons add these intervals above it:

| Quality | Semitone intervals from root |
| --- | --- |
| Major | 0, 4, 7 |
| Minor | 0, 3, 7 |
| Diminished | 0, 3, 6 |
| Augmented | 0, 4, 8 |

The five right-thumb buttons add flat 7, major 7, add 9, add 11, and add 13. Each adds its interval without implicitly adding lower extensions. Chord intervals are fixed semitone distances from the selected root; the scale determines the eight available roots.

If more than one quality is held, the controller plays the union of their notes. For example, major plus minor plays root, minor third, major third, and fifth.

## MIDI and implementation notes

- Enumerate MIDI mode as a USB MIDI device rather than a HID gamepad.
- Send note-on when a note button is pressed and matching note-off when it is released. Continue using the existing button scanning and debounce approach.
- Keep track of which MIDI pitches each button started, so releasing it sends note-offs for those exact pitches even if a modifier or setting changes while it is held. Modifier changes affect only subsequent note presses.
- Account for overlapping chords and the drone sharing a MIDI pitch; releasing one source must not stop a pitch still held by another source.
- Start with one MIDI channel. Pitch bend and modulation from the touch surface are outside this mode's scope.
- Use the display to show at least the selected tonic, scale, base octave, and drone state.

## Decisions to revisit after playing it

1. Confirm that the note-button order and extension-button order feel playable on the hardware.
2. Confirm the scale set. Eight buttons naturally cover seven scale degrees plus the octave; scales with a different number of degrees need a mapping rule.
3. Confirm whether simultaneous chord qualities should continue to combine their pitches.
4. Confirm that a custom drone should stay at its absolute pitch when the tonic or base octave changes.
