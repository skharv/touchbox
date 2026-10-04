//! Button mapping and note ownership for the USB MIDI mode.

const SCALES: [[i16; 8]; 4] = [
    [0, 2, 4, 5, 7, 9, 11, 12], // Major
    [0, 2, 3, 5, 7, 8, 10, 12], // Natural minor
    [0, 2, 3, 5, 7, 9, 10, 12], // Dorian
    [0, 2, 4, 5, 7, 9, 10, 12], // Mixolydian
];

pub const SCALE_NAMES: [&str; 4] = ["MAJOR", "MINOR", "DORIAN", "MIXOLYDIAN"];
pub const NOTE_NAMES: [&str; 12] = [
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B",
];

const NOTE_PINS: [usize; 8] = [1, 2, 0, 3, 4, 7, 5, 6]; // X Y A B RT LT RB LB on MCP2 B
const EXTENSIONS: [i16; 5] = [10, 11, 14, 17, 21]; // b7, maj7, add9, add11, add13
const QUEUE_CAPACITY: usize = 256;
const MAX_CHORD_NOTES: usize = 11;

#[derive(Clone, Copy, Default)]
pub struct Buttons {
    pub a1: [bool; 8],
    pub b1: [bool; 8],
    pub a2: [bool; 8],
    pub b2: [bool; 8],
}

#[derive(Clone, Copy, Default, Debug, Eq, PartialEq)]
pub struct NoteEvent {
    pub note: u8,
    pub on: bool,
}

pub struct MidiState {
    previous: Buttons,
    active: [[u8; MAX_CHORD_NOTES]; 8],
    active_len: [usize; 8],
    owners: [u8; 128],
    queue: [NoteEvent; QUEUE_CAPACITY],
    queue_head: usize,
    queue_len: usize,
    drone_selected_this_hold: bool,
    drone_custom: bool,
    pub drone_on: bool,
    pub drone_note: u8,
    pub tonic: u8,
    pub scale: usize,
    pub base_note: i16,
    pub display_dirty: bool,
}

impl MidiState {
    pub fn new() -> Self {
        Self {
            previous: Buttons::default(),
            active: [[0; MAX_CHORD_NOTES]; 8],
            active_len: [0; 8],
            owners: [0; 128],
            queue: [NoteEvent::default(); QUEUE_CAPACITY],
            queue_head: 0,
            queue_len: 0,
            drone_selected_this_hold: false,
            drone_custom: false,
            drone_on: false,
            drone_note: 48,
            tonic: 0,
            scale: 0,
            base_note: 60,
            display_dirty: true,
        }
    }

    pub fn reset_usb(&mut self) {
        self.previous = Buttons::default();
        self.active_len = [0; 8];
        self.owners = [0; 128];
        self.queue_head = 0;
        self.queue_len = 0;
        self.drone_on = false;
        self.drone_selected_this_hold = false;
        self.display_dirty = true;
    }

    pub fn peek_event(&self) -> Option<NoteEvent> {
        (self.queue_len > 0).then_some(self.queue[self.queue_head])
    }

    pub fn pop_event(&mut self) {
        if self.queue_len > 0 {
            self.queue_head = (self.queue_head + 1) % QUEUE_CAPACITY;
            self.queue_len -= 1;
        }
    }

    fn enqueue(&mut self, note: u8, on: bool) {
        assert!(self.queue_len < QUEUE_CAPACITY, "MIDI event queue overflow");
        let tail = (self.queue_head + self.queue_len) % QUEUE_CAPACITY;
        self.queue[tail] = NoteEvent { note, on };
        self.queue_len += 1;
    }

    fn add_owner(&mut self, note: u8) {
        let owners = &mut self.owners[note as usize];
        if *owners == 0 {
            self.enqueue(note, true);
        }
        self.owners[note as usize] += 1;
    }

    fn remove_owner(&mut self, note: u8) {
        let owners = &mut self.owners[note as usize];
        assert!(*owners > 0);
        *owners -= 1;
        if *owners == 0 {
            self.enqueue(note, false);
        }
    }

    fn root(&self, degree: usize, buttons: &Buttons) -> i16 {
        let accidental = i16::from(buttons.a1[5]) - i16::from(buttons.a1[4]);
        self.base_note
            + i16::from(self.tonic)
            + SCALES[self.scale][degree]
            + accidental
            + if buttons.a1[6] { 12 } else { 0 }
    }

    fn add_unique(notes: &mut [u8; MAX_CHORD_NOTES], len: &mut usize, pitch: i16) {
        if (0..=127).contains(&pitch) {
            let pitch = pitch as u8;
            if !notes[..*len].contains(&pitch) {
                notes[*len] = pitch;
                *len += 1;
            }
        }
    }

    fn start_note(&mut self, degree: usize, buttons: &Buttons) {
        let root = self.root(degree, buttons);
        let mut notes = [0; MAX_CHORD_NOTES];
        let mut len = 0;
        Self::add_unique(&mut notes, &mut len, root);

        // Major, minor, diminished, augmented; simultaneous qualities union their pitches.
        let qualities = [(0, [4, 7]), (1, [3, 7]), (2, [3, 6]), (3, [4, 8])];
        for (pin, intervals) in qualities {
            if buttons.a1[pin] {
                for interval in intervals {
                    Self::add_unique(&mut notes, &mut len, root + interval);
                }
            }
        }
        for (pin, interval) in EXTENSIONS.iter().enumerate() {
            if buttons.a2[pin] {
                Self::add_unique(&mut notes, &mut len, root + interval);
            }
        }

        self.active[degree] = notes;
        self.active_len[degree] = len;
        for note in notes[..len].iter().copied() {
            self.add_owner(note);
        }
    }

    fn stop_note(&mut self, degree: usize) {
        for index in 0..self.active_len[degree] {
            self.remove_owner(self.active[degree][index]);
        }
        self.active_len[degree] = 0;
    }

    fn change_drone(&mut self, new_note: u8) {
        if self.drone_on && self.drone_note != new_note {
            self.remove_owner(self.drone_note);
            self.add_owner(new_note);
        }
        self.drone_note = new_note;
        self.display_dirty = true;
    }

    fn sync_default_drone(&mut self) {
        if !self.drone_custom {
            self.change_drone((self.base_note + i16::from(self.tonic) - 12) as u8);
        }
    }

    pub fn update(&mut self, buttons: Buttons) {
        let rising = |now: bool, before: bool| now && !before;

        if rising(buttons.b1[0], self.previous.b1[0]) {
            self.scale = (self.scale + SCALES.len() - 1) % SCALES.len();
            self.display_dirty = true;
        }
        if rising(buttons.b1[1], self.previous.b1[1]) {
            self.scale = (self.scale + 1) % SCALES.len();
            self.display_dirty = true;
        }
        if rising(buttons.a2[5], self.previous.a2[5]) && self.base_note > 36 {
            self.base_note -= 12;
            self.sync_default_drone();
            self.display_dirty = true;
        }
        if rising(buttons.a2[6], self.previous.a2[6]) && self.base_note < 72 {
            self.base_note += 12;
            self.sync_default_drone();
            self.display_dirty = true;
        }
        if rising(buttons.a2[7], self.previous.a2[7]) {
            self.drone_selected_this_hold = false;
        }

        // Finish released notes before starting new ones, preserving ownership of overlaps.
        for (degree, pin) in NOTE_PINS.iter().copied().enumerate() {
            if !buttons.b2[pin] && self.previous.b2[pin] {
                self.stop_note(degree);
            }
        }

        for (degree, pin) in NOTE_PINS.iter().copied().enumerate() {
            if !rising(buttons.b2[pin], self.previous.b2[pin]) {
                continue;
            }
            let root = self.root(degree, &buttons);
            if buttons.a1[7] {
                self.tonic = root.rem_euclid(12) as u8;
                self.sync_default_drone();
                self.display_dirty = true;
            } else if buttons.a2[7] {
                self.drone_selected_this_hold = true;
                self.drone_custom = true;
                let drone_pitch = root - 12;
                if (0..=127).contains(&drone_pitch) {
                    self.change_drone(drone_pitch as u8);
                }
            } else {
                self.start_note(degree, &buttons);
            }
        }

        if !buttons.a2[7] && self.previous.a2[7] && !self.drone_selected_this_hold {
            self.drone_on = !self.drone_on;
            if self.drone_on {
                self.add_owner(self.drone_note);
            } else {
                self.remove_owner(self.drone_note);
            }
            self.display_dirty = true;
        }

        self.previous = buttons;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(state: &mut MidiState) -> NoteEvent {
        let event = state.peek_event().unwrap();
        state.pop_event();
        event
    }

    #[test]
    fn note_off_uses_original_pitch_after_modifier_changes() {
        let mut state = MidiState::new();
        let mut buttons = Buttons::default();
        buttons.b2[NOTE_PINS[0]] = true;
        state.update(buttons);
        assert_eq!(event(&mut state), NoteEvent { note: 60, on: true });
        buttons.a1[5] = true;
        state.update(buttons);
        buttons.b2[NOTE_PINS[0]] = false;
        state.update(buttons);
        assert_eq!(
            event(&mut state),
            NoteEvent {
                note: 60,
                on: false
            }
        );
    }

    #[test]
    fn overlapping_drone_does_not_stop_held_note() {
        let mut state = MidiState::new();
        state.drone_note = 60;
        let mut buttons = Buttons::default();
        buttons.b2[NOTE_PINS[0]] = true;
        state.update(buttons);
        assert_eq!(event(&mut state), NoteEvent { note: 60, on: true });
        buttons.a2[7] = true;
        state.update(buttons);
        buttons.a2[7] = false;
        state.update(buttons);
        assert!(state.peek_event().is_none());
        buttons.b2[NOTE_PINS[0]] = false;
        state.update(buttons);
        assert!(state.peek_event().is_none());
        buttons.a2[7] = true;
        state.update(buttons);
        buttons.a2[7] = false;
        state.update(buttons);
        assert_eq!(
            event(&mut state),
            NoteEvent {
                note: 60,
                on: false
            }
        );
    }

    #[test]
    fn hold_drone_and_select_does_not_toggle() {
        let mut state = MidiState::new();
        let mut buttons = Buttons::default();
        buttons.a2[7] = true;
        state.update(buttons);
        buttons.b2[NOTE_PINS[2]] = true;
        state.update(buttons);
        assert_eq!(state.drone_note, 52);
        buttons.a2[7] = false;
        state.update(buttons);
        assert!(!state.drone_on);
        assert!(state.peek_event().is_none());
    }

    #[test]
    fn major_minor_and_ninth_combine_without_duplicate_root() {
        let mut state = MidiState::new();
        let mut buttons = Buttons::default();
        buttons.a1[0] = true;
        buttons.a1[1] = true;
        buttons.a2[2] = true;
        buttons.b2[NOTE_PINS[0]] = true;
        state.update(buttons);
        let mut notes = [0; 5];
        for note in &mut notes {
            *note = event(&mut state).note;
        }
        notes.sort_unstable();
        assert_eq!(notes, [60, 63, 64, 67, 74]);
        assert!(state.peek_event().is_none());
    }

    #[test]
    fn changing_key_moves_default_drone_but_not_custom_drone() {
        let mut state = MidiState::new();
        let mut buttons = Buttons::default();
        buttons.a1[7] = true;
        state.update(buttons);
        buttons.b2[NOTE_PINS[1]] = true;
        state.update(buttons);
        assert_eq!(state.tonic, 2);
        assert_eq!(state.drone_note, 50);
        assert!(state.peek_event().is_none());

        buttons.b2[NOTE_PINS[1]] = false;
        buttons.a1[7] = false;
        buttons.a2[7] = true;
        state.update(buttons);
        buttons.b2[NOTE_PINS[2]] = true;
        state.update(buttons);
        assert_eq!(state.drone_note, 54);

        buttons.b2[NOTE_PINS[2]] = false;
        buttons.a2[7] = false;
        buttons.a1[7] = true;
        state.update(buttons);
        buttons.b2[NOTE_PINS[1]] = true;
        state.update(buttons);
        assert_eq!(state.drone_note, 54);
    }
}
