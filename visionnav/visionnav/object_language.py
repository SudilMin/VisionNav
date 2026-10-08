#!/usr/bin/env python3
"""
object_language.py
==================
Spoken references to mapped objects, for a user who cannot see the map.

A blind user cannot know that the table they want is "table_2". They say what they know about it:
"the table where the cup is", "the chair next to the door", "the nearest chair", "my chair".
This module turns such a request into the matching objects of the object map (/semantic_objects from
object_perception), and describes objects back in terms a blind person can use without sight:
where it is from them (feet and clock direction), what it rests on or stands next to, which saved room
it is in, and names they gave things themselves ("your chair"). Object IDs are never spoken. Colours are
understood when someone says one (a helper, a partially sighted user) but only spoken if asked for
(speak_colors), since someone blind from birth may not know them.

Relations are worked out from the 3-D map, not from a single camera frame:
  on      a small object resting on a table, shelf, counter...   (its base at the support's top height)
  holds   the reverse: a table with a cup on it
  near    within about a metre of each other
  under   below a table's top, inside its footprint

Pure Python (no ROS), so it can be tested on its own.
"""

import math
import re

# Colour words object_perception reports, plus spoken variants
COLORS = {"red", "orange", "yellow", "green", "blue", "purple", "pink", "brown", "black", "white", "grey"}
COLOR_ALIASES = {"gray": "grey", "violet": "purple", "dark": None, "light": None}

# Spoken words for object classes (the map uses object_perception's reported names)
CLASS_ALIASES = {
    "mug": "cup", "glass": "cup", "coffee cup": "cup", "tea cup": "cup",
    "desk": "table", "dining table": "table", "coffee table": "table", "side table": "table",
    "couch": "sofa", "settee": "sofa", "cupboard": "cabinet", "almirah": "wardrobe",
    "switch": "light switch", "light switch board": "light switch", "socket": "wall socket",
    "plug point": "wall socket", "power socket": "wall socket", "outlet": "wall socket",
    "tv": "monitor", "television": "monitor", "screen": "monitor", "phone": "smartphone",
    "mobile": "smartphone", "mobile phone": "smartphone", "cell phone": "smartphone",
    "fridge": "refrigerator", "bin": "trash can", "dustbin": "trash can", "garbage bin": "trash can",
    "doorway": "door", "entrance": "door", "plant": "plant", "potted plant": "plant",
    "stairs": "stairs", "staircase": "stairs", "steps": "stairs", "water bottle": "bottle",
    # tools and small things on a table (hand mode)
    "screw driver": "screwdriver", "spanner": "wrench", "torch": "flashlight", "tape": "adhesive tape",
    "sellotape": "adhesive tape", "scotch tape": "adhesive tape", "measuring tape": "tape measure",
    "earbuds": "earphones", "charger": "phone charger", "spectacles": "glasses", "specs": "glasses",
    "cellphone": "smartphone",
}

# Things other objects rest on
SUPPORTS = {"table", "shelf", "bookshelf", "cabinet", "kitchen counter", "bed", "sofa", "chair", "stool",
            "tv stand", "dressing table", "chest of drawers", "sink", "refrigerator", "microwave", "box",
            "bench", "wardrobe", "drawer", "washing machine"}
# Things worth naming as a landmark ("the chair next to the door")
LANDMARKS = ["door", "window", "table", "sofa", "bed", "refrigerator", "wardrobe", "cabinet", "sink",
             "kitchen counter", "shelf", "bookshelf", "monitor", "stairs", "chair", "plant", "trash can"]
# Big furniture: its colour is only mentioned when it tells two of them apart
FURNITURE = {"table", "chair", "sofa", "bed", "wardrobe", "cabinet", "shelf", "bookshelf", "door", "window",
             "kitchen counter", "chest of drawers", "tv stand", "dressing table", "stool", "bench"}

# Seats: things rest on the seat, not on the top of the backrest the map's height measures
SEATS = {"chair", "sofa", "stool", "bench"}
SEAT_HEIGHT = 0.5      # m
WIDE_SUPPORTS = {"table", "bed", "kitchen counter", "dressing table", "desk"}  # drawn at least 0.8 m wide
ON_Z_TOL = 0.35        # m: an object's base within this of the support's top height is resting on it
ON_XY_MARGIN = 0.25    # m beyond the support's half width
MIN_ELEVATION = 0.25   # m: an object this high off the floor is not standing on the floor
NEAR_GAP = 1.0         # m between the two objects' edges
LANDMARK_GAP = 1.5     # m: landmarks this close can describe an object
PLACE_RADIUS = 3.0     # m: an object this close to a saved place is "in the kitchen" / "near the front door"
ROOM_WORDS = ("kitchen", "bedroom", "bathroom", "toilet", "living room", "dining room", "hall", "office",
              "room", "lounge", "garage", "balcony", "study", "corridor", "veranda", "store room")
M_TO_FT = 3.28084

HOLDS_WORDS = ("with", "has", "having", "holding", "where", "carrying", "which has", "that has", "containing",
               "on which", "on whose")
ON_WORDS = ("on top of", "on", "above", "upon", "over", "sitting on", "lying on", "kept on", "placed on")
NEAR_WORDS = ("next to", "near", "beside", "by", "close to", "nearby", "in front of", "behind", "at")
UNDER_WORDS = ("under", "below", "beneath", "underneath")
ORDINALS = {"first": 0, "1st": 0, "one": 0, "second": 1, "2nd": 1, "two": 1, "third": 2, "3rd": 2,
            "three": 2, "fourth": 3, "4th": 3, "four": 3, "last": -1}


# ─────────────────────────────── geometry ───────────────────────────────
def _half(o, minimum=0.1):
    return max(minimum, 0.5 * float(o.get("w", 0.3)))


def _gap(a, b):
    """Distance between two objects' footprint edges (m)."""
    return max(0.0, math.hypot(a["x"] - b["x"], a["y"] - b["y"]) - _half(a) - _half(b))


def _top(b):
    top = b.get("z", 0.0) + b.get("h", 0.0)
    return min(top, b.get("z", 0.0) + SEAT_HEIGHT) if b["class"] in SEATS else top


def _could_rest_on(a, b):
    if a is b or b["class"] not in SUPPORTS or a["class"] in SUPPORTS and a["class"] != "box":
        return False
    if a.get("z", 0.0) < MIN_ELEVATION:
        return False
    half = _half(b, 0.4 if b["class"] in WIDE_SUPPORTS else 0.1) + ON_XY_MARGIN
    return (abs(a["x"] - b["x"]) <= half and abs(a["y"] - b["y"]) <= half
            and abs(a.get("z", 0.0) - _top(b)) <= ON_Z_TOL)


def support_of(a, objects):
    """The one thing `a` rests on (a cup by a table and a chair is on the table): best matching top height,
    then nearest centre. None if it rests on nothing on the map."""
    cands = [b for b in objects if _could_rest_on(a, b)]
    if not cands:
        return None
    return min(cands, key=lambda b: abs(a.get("z", 0.0) - _top(b)) + 0.5 * math.hypot(a["x"] - b["x"], a["y"] - b["y"]))


def is_on(a, b, objects=None):
    """a rests on b (a cup on a table)."""
    if objects is None:
        return _could_rest_on(a, b)
    return _could_rest_on(a, b) and support_of(a, objects) is b


def is_under(a, b):
    """a is under b (a box under the table)."""
    if a is b or b["class"] not in {"table", "desk", "bed", "chair", "bench", "shelf", "sink"}:
        return False
    half = _half(b, 0.4)
    return (abs(a["x"] - b["x"]) <= half and abs(a["y"] - b["y"]) <= half
            and a.get("z", 0.0) + a.get("h", 0.0) <= b.get("z", 0.0) + b.get("h", 0.0) + 0.05
            and a.get("z", 0.0) < MIN_ELEVATION)


def is_near(a, b):
    return a is not b and _gap(a, b) <= NEAR_GAP


RELATIONS = {"on": is_on, "holds": lambda a, b, objs: is_on(b, a, objs),
             "under": lambda a, b, objs: is_under(a, b), "near": lambda a, b, objs: is_near(a, b)}


# ─────────────────────────────── parsing ───────────────────────────────
class Mention:
    def __init__(self, cls, color, start, end):
        self.cls, self.color, self.start, self.end = cls, color, start, end

    def __repr__(self):
        return f"Mention({self.color or ''} {self.cls})"


class Query:
    """What the user asked for: target class and colour, relations to other objects, and a selector."""
    def __init__(self, target=None, relations=None, selector=None):
        self.target = target              # Mention
        self.relations = relations or []  # [(relation, Mention)]
        self.selector = selector          # "nearest" | "farthest" | "left" | "right" | int index

    def __repr__(self):
        return f"Query({self.target}, {self.relations}, sel={self.selector})"


def _normalise(text):
    text = text.lower().replace("-", " ")
    text = re.sub(r"[^a-z0-9 ]", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def _lexicon(known_classes):
    """Spoken phrase -> class, including plurals, for the classes on the map and the aliases."""
    lex = {}
    for cls in set(known_classes) | set(CLASS_ALIASES.values()):
        lex[cls] = cls
    for alias, cls in CLASS_ALIASES.items():
        lex[alias] = cls
    for phrase, cls in list(lex.items()):
        if phrase.endswith(("s", "sh", "ch", "x")):
            lex.setdefault(phrase + "es", cls)
        elif phrase.endswith("f"):
            lex.setdefault(phrase[:-1] + "ves", cls)
        else:
            lex.setdefault(phrase + "s", cls)
    return lex


def _mentions(words, lex):
    """Object mentions in word order, each with the colour word just before it."""
    out, i = [], 0
    longest = max((len(p.split()) for p in lex), default=1)
    while i < len(words):
        for n in range(min(longest, len(words) - i), 0, -1):
            phrase = " ".join(words[i:i + n])
            if phrase in lex:
                color, j = None, i - 1
                while j >= 0 and words[j] in ("the", "a", "an", "that", "this", "coloured", "colored"):
                    j -= 1
                if j >= 0 and (words[j] in COLORS or words[j] in COLOR_ALIASES):
                    color = COLOR_ALIASES.get(words[j], words[j])
                # "the cup that is red", "the red coloured cup"
                k = i + n
                if color is None and words[k:k + 2] in (["that", "is"], ["which", "is"]) and k + 2 < len(words):
                    if words[k + 2] in COLORS or words[k + 2] in COLOR_ALIASES:
                        color = COLOR_ALIASES.get(words[k + 2], words[k + 2])
                out.append(Mention(lex[phrase], color, i, i + n))
                i += n
                break
        else:
            i += 1
    return out


def _has(phrase_words, candidates):
    text = " " + " ".join(phrase_words) + " "
    return next((c for c in candidates if f" {c} " in text), None)


def _relation(between_words, after_words):
    """Which relation the words linking the target to another object express."""
    if _has(between_words, HOLDS_WORDS):
        return "holds"
    for words in (between_words, after_words):
        if _has(words, UNDER_WORDS):
            return "under"
        if _has(words, NEAR_WORDS) and not _has(words, ("on top of",)):
            return "near"
        if _has(words, ON_WORDS):
            return "on"
    return "near"


def _selector(words):
    text = " " + " ".join(words) + " "
    if any(w in text for w in (" nearest ", " closest ", " near me ", " closer ")):
        return "nearest"
    if any(w in text for w in (" farthest ", " furthest ", " far one ")):
        return "farthest"
    if re.search(r" (my|the) left ", text) or " left one " in text or " left side " in text:
        return "left"
    if re.search(r" (my|the) right ", text) or " right one " in text or " right side " in text:
        return "right"
    for w in words:
        if w in ORDINALS and w not in ("one",):
            return ORDINALS[w]
    return None


def parse(text, known_classes, default_class=None):
    """Parse a spoken object reference. `default_class` stands in for "the one ..." (a follow-up answer)."""
    words = _normalise(text).split()
    lex = _lexicon(list(known_classes) + ([default_class] if default_class else []))
    ms = _mentions(words, lex)
    selector = _selector(words)
    if not ms or (default_class and ms[0].cls != default_class):
        if default_class is None:
            return Query(selector=selector)
        # "the one next to the door" / "the red one": the target is the class being asked about
        color = next((COLOR_ALIASES.get(w, w) for w in words[:ms[0].start if ms else len(words)]
                      if w in COLORS or w in COLOR_ALIASES), None)
        ms = [Mention(default_class, color, 0, 0)] + ms
    target, rels = ms[0], []
    for k, m in enumerate(ms[1:], 1):
        between = words[ms[k - 1].end:m.start]
        after = words[m.end:ms[k + 1].start] if k + 1 < len(ms) else words[m.end:]
        rels.append((_relation(between, after), m))
    return Query(target, rels, selector)


# ─────────────────────────────── resolving ───────────────────────────────
def _color_ok(obj, color):
    return color is None or obj.get("color") in (None, color)


def _bearing(obj, pose):
    """(distance m, angle rad; + is left) of an object from the user."""
    rx, ry, yaw = pose
    ang = math.atan2(obj["y"] - ry, obj["x"] - rx) - yaw
    return math.hypot(obj["x"] - rx, obj["y"] - ry), math.atan2(math.sin(ang), math.cos(ang))


def resolve(query, objects, pose=None):
    """Objects matching the query, nearest first (ties by exact colour matches)."""
    if query.target is None:
        return []
    t = query.target
    cands = [o for o in objects if o["class"] == t.cls and _color_ok(o, t.color)]
    for rel, anchor in query.relations:
        test = RELATIONS[rel]
        anchors = [o for o in objects if o["class"] == anchor.cls and _color_ok(o, anchor.color)]
        cands = [c for c in cands if any(test(c, a, objects) for a in anchors)]
    exact = lambda o: 0 if (t.color is None or o.get("color") == t.color) else 1  # noqa: E731
    if pose is not None:
        cands.sort(key=lambda o: (exact(o), _bearing(o, pose)[0]))
    else:
        cands.sort(key=exact)
    sel = query.selector
    if sel is None or not cands:
        return cands
    if pose is not None and sel in ("nearest", "farthest", "left", "right"):
        if sel == "nearest":
            return cands[:1]
        if sel == "farthest":
            return cands[-1:]
        side = [c for c in cands if (_bearing(c, pose)[1] > 0) == (sel == "left")]
        return side[:1] or cands[:1]
    if isinstance(sel, int):
        return [cands[sel]] if -len(cands) <= sel < len(cands) else []
    return cands


# ─────────────────────────────── describing ───────────────────────────────
def _article_name(o, with_color):
    c = o.get("color")
    return f"{c} {o['class']}" if with_color and c else o["class"]


def _salient(o):
    """Small, coloured objects make the best landmarks on a support."""
    return (o["class"] in FURNITURE, o.get("color") is None)


def place_phrase(o, places):
    """ "in the kitchen" / "near the front door": the nearest saved place within PLACE_RADIUS, or ""."""
    if not places:
        return ""
    name, p = min(places.items(), key=lambda kv: math.hypot(kv[1]["x"] - o["x"], kv[1]["y"] - o["y"]))
    if math.hypot(p["x"] - o["x"], p["y"] - o["y"]) > PLACE_RADIUS:
        return ""
    return f"in the {name}" if any(w in name for w in ROOM_WORDS) else f"near the {name}"


def context_options(o, objects, places=None, colors=False):
    """Phrases placing an object among others, best first: "with the cup on it", "on the table",
    "in the kitchen", "next to the door", "next to the light switch"."""
    opts = [f"with the {_article_name(x, colors)} on it"
            for x in sorted((x for x in objects if is_on(x, o, objects)), key=_salient)]
    support = support_of(o, objects)
    if support is not None:
        opts.append(f"on the {_article_name(support, colors and support['class'] not in FURNITURE)}")
    room = place_phrase(o, places)
    if room:
        opts.append(room)
    near = sorted((x for x in objects if x["class"] != o["class"] and not x.get("dynamic")
                   and _gap(o, x) <= LANDMARK_GAP and x is not support),
                  key=lambda x: (LANDMARKS.index(x["class"]) if x["class"] in LANDMARKS else len(LANDMARKS),
                                 _gap(o, x)))
    for x in near:
        phrase = f"next to the {x['class']}"
        if phrase not in opts:
            opts.append(phrase)
    return opts


def context(o, objects, others=(), places=None, colors=False):
    """The phrase that best tells `o` apart from `others` (same-class candidates), or "" if none."""
    opts = context_options(o, objects, places, colors)
    taken = {p for x in others for p in context_options(x, objects, places, colors)}
    return next((p for p in opts if p not in taken), opts[0] if opts else "")


def where(o, pose):
    """Direction and distance from the user: "6 feet away, at 1 o'clock"."""
    if pose is None:
        return ""
    dist, ang = _bearing(o, pose)
    ft = dist * M_TO_FT
    clock = int(round(12 - ang * 6 / math.pi)) % 12 or 12
    if ft < 2:
        return "right in front of you" if clock in (11, 12, 1) else f"right beside you, at {clock} o'clock"
    feet = int(round(ft)) if ft < 30 else int(round(ft / 5.0)) * 5
    return f"{feet} feet away, at {clock} o'clock"


def label_of(o, labels):
    """The user's own name for an object ("my chair" -> "your chair"), or None."""
    name = (labels or {}).get(o.get("name"))
    if not name:
        return None
    return "your " + name[3:] if name.startswith("my ") else f"the {name}"


def describe(o, objects, pose=None, others=(), places=None, labels=None, colors=False):
    """Spoken description of an object: its name, what tells it apart, and where it is from the user.

    `others` are the other candidates it must be told apart from. With `colors`, colour is mentioned when it
    differs from theirs (and always for small objects).
    """
    own = label_of(o, labels)
    if own:
        parts = [own]
    else:
        tell = {x.get("color") for x in others}
        with_color = colors and o.get("color") is not None and (
            o["class"] not in FURNITURE or o.get("color") not in tell)
        parts = [f"the {_article_name(o, with_color)}"]
        ctx = context(o, objects, others, places, colors)
        if ctx:
            parts.append(ctx)
    loc = where(o, pose)
    if loc:
        parts.append(loc)
    if not o.get("live", True) and o.get("seen_ago", 0) > 30:
        mins = int(o["seen_ago"] // 60)
        parts.append(f"last seen {mins} minute{'s' if mins != 1 else ''} ago" if mins else "seen a moment ago")
    return ", ".join(parts)


def spoken_name(o, objects, places=None, labels=None, colors=False):
    """Short name for navigation speech: "the table with the cup on it", "your chair"."""
    own = label_of(o, labels)
    if own:
        return own
    with_color = colors and o.get("color") is not None and o["class"] not in FURNITURE
    others = [x for x in objects if x["class"] == o["class"] and x is not o]
    ctx = context(o, objects, others, places, colors)
    return f"the {_article_name(o, with_color)}" + (f" {ctx}" if ctx else "")


def query_text(query):
    """The request read back in words, for "I have not seen ..." answers."""
    t = query.target
    text = f"{t.color + ' ' if t.color else ''}{t.cls}"
    words = {"on": "on", "holds": "with", "near": "next to", "under": "under"}
    for rel, m in query.relations:
        anchor = f"{m.color + ' ' if m.color else ''}{m.cls}"
        text += f" {words[rel]} a {anchor}" + (" on it" if rel == "holds" else "")
    return text


NEXT_WORDS = ("another one", "another", "next one", "next", "the other one", "other one", "a different one",
              "different one", "not that one", "the next one", "some other one")


def is_next(text):
    """ "another one": move on to the next nearest match."""
    return " ".join(_normalise(text).split()) in NEXT_WORDS


def is_selection(text):
    """A follow-up answer choosing among offered candidates ("the first one", "the one next to the door")."""
    words = _normalise(text).split()
    return (any(w in ORDINALS for w in words) or "one" in words
            or _selector(words) is not None or words[:1] in (["that"], ["this"]))
