#!/usr/bin/env python3
"""
speech_fix.py
=============
Repairs what speech recognition mishears in the few words this system listens for. Whisper writes what it hears
as ordinary English, so a short command or an object's name comes out as a sound-alike: "chair one" -> "share one",
"stop" -> "top", "Kamal" -> "camel", "Nimal" -> "nimble" (measured on recordings of the system's own commands).
Here a word that is not one the system knows is replaced by a known word that sounds the same (same consonant
skeleton: "share" and "chair" are both X-R), and a whole short sentence close to a command becomes that command.
Plain Python, no models.
"""

import difflib
import re

# Never "corrected": they carry the sentence, not a name
COMMON = set("""a an the to of on in at by for with and or is are was it this that there here my your his her their
me you him them i we what where which who how why can could do does go take bring walk lead guide find look show
tell say save call name named forget remember place near next behind front left right side top bottom one two three
four five six seven eight nine ten first second third nearest another other big small red green blue yellow white
black brown grey gray orange pink purple colour color please now again""".split())


def sound_key(word: str) -> str:
    """Consonant skeleton of how a word sounds: "share" and "chair" -> "xr", "camel" and "Kamal" -> "kml"."""
    w = re.sub(r"[^a-z]", "", word.lower())
    if not w:
        return ""
    for a, b in (("tch", "x"), ("sch", "sk"), ("ch", "x"), ("sh", "x"), ("ph", "f"), ("ck", "k"), ("gh", ""),
                 ("kn", "n"), ("wr", "r"), ("dg", "j"), ("qu", "kw"), ("wh", "w"), ("th", "0")):
        w = w.replace(a, b)
    if not w:
        return ""
    w = re.sub(r"c(?=[eiy])", "s", w).replace("c", "k").replace("q", "k").replace("z", "s").replace("v", "f")
    key = w[0] + re.sub(r"[aeiouyhw]", "", w[1:])
    key = re.sub(r"(.)\1+", r"\1", key)
    return ("a" + key[1:]) if key[0] in "aeiouy" else key


def _close_keys(a: str, b: str) -> bool:
    """Same skeleton, or (3 consonants or more) one consonant added, dropped or changed: "nimble"/"Nimal"."""
    if a == b:
        return True
    if min(len(a), len(b)) < 3 or abs(len(a) - len(b)) > 1:
        return False
    prev = list(range(len(b) + 1))  # edit distance, row by row
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1] <= 1


def fix_words(text: str, vocabulary, protect=()) -> str:
    """Replace each word that is not known (`vocabulary`, COMMON) by the known word it sounds like, if exactly one
    does (the same skeleton first, else one consonant apart). "go to the share one", "chair" known -> "go to the
    chair one". Words in `protect` (the commands' own words: "status", "vision", "cross") are never replaced:
    one consonant apart, "status" was "stool" and "vision" was "Kasun". A near match must also start alike."""
    known = {}
    for v in vocabulary:
        for w in str(v).lower().replace("_", " ").split():
            if len(w) >= 3 and w not in COMMON:
                known.setdefault(sound_key(w), set()).add(w)
    if not known:
        return text
    protect = {w for p in protect for w in str(p).lower().split()}
    words = text.split()
    for i, w in enumerate(words):
        if w in COMMON or w in protect or w.isdigit() or any(w in ws for ws in known.values()) or len(w) < 3:
            continue
        k = sound_key(w)
        hits = set(known.get(k, ())) or {v for key, vs in known.items()
                                         if key[:1] == k[:1] and _close_keys(k, key) for v in vs}
        if len(hits) == 1:
            words[i] = hits.pop()
    return " ".join(words)


def fix_command(text: str, phrases) -> str:
    """A short sentence (up to 3 words) that is not a command but sounds like exactly one: that command ("top" ->
    "stop", "vision of" -> "vision off")."""
    if text in phrases or not text or len(text.split()) > 3:
        return text
    keys = {p: " ".join(sound_key(w) for w in p.split()) for p in phrases}
    mine = " ".join(sound_key(w) for w in text.split())
    same = [p for p, k in keys.items() if k == mine]
    if len(same) == 1:
        return same[0]
    close = difflib.get_close_matches(text, list(phrases), n=2, cutoff=0.8)
    return close[0] if len(close) == 1 else text
