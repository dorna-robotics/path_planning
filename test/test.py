# test_fcl_box_semantics.py
import numpy as np

import fcl

def make_tf(x, y, z):
    # Works across common python-fcl builds
    if hasattr(fcl, "Transform"):
        try:
            return fcl.Transform(np.eye(3), np.array([x, y, z], dtype=float))
        except TypeError:
            return fcl.Transform([0, 0, 0, 1], [x, y, z])  # quat + translation
    if hasattr(fcl, "Transform3f"):
        try:
            return fcl.Transform3f(np.eye(3), np.array([x, y, z], dtype=np.float32))
        except TypeError:
            return fcl.Transform3f([0, 0, 0, 1], [x, y, z])
    if hasattr(fcl, "Transformd"):
        try:
            return fcl.Transformd(np.eye(3), np.array([x, y, z], dtype=float))
        except TypeError:
            return fcl.Transformd([0, 0, 0, 1], [x, y, z])
    raise RuntimeError("No Transform class found in fcl module")

def collide_bool(o1, o2):
    # Try the “standard” API first
    if hasattr(fcl, "CollisionRequest") and hasattr(fcl, "CollisionResult"):
        req = fcl.CollisionRequest()
        res = fcl.CollisionResult()
        fcl.collide(o1, o2, req, res)

        # Different bindings expose different result accessors
        if hasattr(res, "is_collision"):
            return bool(res.is_collision)
        if hasattr(res, "isCollision"):
            return bool(res.isCollision())
        if hasattr(res, "numContacts"):
            return res.numContacts() > 0
        if hasattr(res, "contacts"):
            return len(res.contacts) > 0

        # If none of the above exist, fall through and try other styles
    # Some bindings return an integer contact count directly
    out = fcl.collide(o1, o2)
    return bool(out)

def collision_threshold_for_box(box_dims, lo=0.0, hi=5.0, iters=60):
    """
    Binary search for the separation along +X where two identical boxes
    transition from collision->no collision (touching threshold).
    """
    boxA = fcl.Box(*box_dims)
    boxB = fcl.Box(*box_dims)

    # One at origin, one at +x
    o1 = fcl.CollisionObject(boxA, make_tf(0.0, 0.0, 0.0))

    # Ensure hi is non-colliding (expand until it is)
    x = hi
    while True:
        o2 = fcl.CollisionObject(boxB, make_tf(x, 0.0, 0.0))
        if not collide_bool(o1, o2):
            break
        x *= 2.0
        if x > 1e6:
            raise RuntimeError("Could not find a non-colliding separation; something is wrong.")

    hi = x
    lo = 0.0

    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        o2 = fcl.CollisionObject(boxB, make_tf(mid, 0.0, 0.0))
        if collide_bool(o1, o2):
            lo = mid
        else:
            hi = mid

    return 0.5 * (lo + hi)

def main():
    print("fcl module:", fcl)
    print("Has Box:", hasattr(fcl, "Box"))
    print("Has CollisionRequest/Result:", hasattr(fcl, "CollisionRequest"), hasattr(fcl, "CollisionResult"))
    print("Has Transform variants:", hasattr(fcl, "Transform"), hasattr(fcl, "Transform3f"), hasattr(fcl, "Transformd"))

    dims = (1.0, 1.0, 1.0)
    thr = collision_threshold_for_box(dims)
    print(f"\nBox dims args = {dims}")
    print(f"Estimated touch threshold along X = {thr:.9f}")

    # Interpretation:
    # If Box(1,1,1) uses FULL side lengths, threshold should be ~1.0
    # If Box(1,1,1) uses HALF-lengths/extents, threshold should be ~2.0
    if abs(thr - 1.0) < 1e-3:
        print("Conclusion: Box(a,b,c) uses FULL side lengths.")
    elif abs(thr - 2.0) < 1e-3:
        print("Conclusion: Box(a,b,c) uses HALF-lengths/extents.")
    else:
        print("Conclusion unclear (threshold not near 1 or 2). Check units/transforms or contact settings.")

if __name__ == "__main__":
    main()
