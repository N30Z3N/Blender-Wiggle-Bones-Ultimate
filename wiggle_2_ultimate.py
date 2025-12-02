bl_info = {
    "name": "Wiggle 2 (Collision Stability Fix)",
    "author": "Steve Miller, Labhatorian, Fixed by Elite Technologist",
    "version": (3, 2, 0),
    "blender": (4, 0, 0),
    "location": "3d Viewport > Animation Panel",
    "description": "Simulate physics on Bones with Stabilized Collision System",
    "warning": "",
    "wiki_url": "https://github.com/shteeve3d/blender-wiggle-2",
    "category": "Animation",
}

import bpy
from mathutils import Vector, Matrix, Euler, Quaternion
from bpy.app.handlers import persistent
import math

# ------------------------------------------------------------------------
#    CONSTANTS & UTILS
# ------------------------------------------------------------------------

ZERO_VEC = Vector((0, 0, 0))
ONE_VEC = Vector((1, 1, 1))

# --- STABILITY CONSTANTS ---
MAX_VELOCITY = 5.0           # Reduced from 10.0
MAX_DISPLACEMENT = 0.5       # Maximum position change per frame
MAX_PUSH = 0.2               # Maximum collision push per frame
MIN_SCALE = 0.5              # Tightened from 0.1
MAX_SCALE = 1.5              # Tightened from 2.0
VELOCITY_DECAY_ON_COLLISION = 0.3  # Kill most velocity on collision


def clamp_vector(vec: Vector, max_length: float) -> Vector:
    """Clamp vector magnitude to max_length."""
    length = vec.length
    if length > max_length and length > 0.0001:
        return vec * (max_length / length)
    return vec.copy()


def is_valid_vector(vec: Vector) -> bool:
    """Check vector for NaN or Inf values."""
    for v in vec:
        if math.isnan(v) or math.isinf(v):
            return False
    return True


def sanitize_vector(vec: Vector, fallback: Vector = None) -> Vector:
    """Return fallback if vector contains invalid values."""
    if fallback is None:
        fallback = ZERO_VEC
    return vec.copy() if is_valid_vector(vec) else fallback.copy()


def relative_matrix(m1: Matrix, m2: Matrix) -> Matrix:
    """Compute relative matrix between two matrices."""
    try:
        return (m2.inverted() @ m1).inverted()
    except ValueError:
        return Matrix.Identity(4)


def flatten(mat: Matrix) -> list:
    """Flatten matrix to list for storage."""
    dim = len(mat)
    return [mat[j][i] for i in range(dim) for j in range(dim)]


def unflatten(flat: list) -> Matrix:
    """Reconstruct matrix from flattened list."""
    mat = Matrix.Identity(4)
    for i in range(4):
        for j in range(4):
            mat[j][i] = flat[i * 4 + j]
    return mat


def decompose_matrix_no_scale(mat: Matrix) -> Matrix:
    """Extract location and rotation from matrix, discard scale."""
    loc, rot, _ = mat.decompose()
    return Matrix.Translation(loc) @ rot.to_matrix().to_4x4()


def reset_scene():
    if not hasattr(bpy.context.scene, "wiggle"):
        return
    for wo in bpy.context.scene.wiggle.list:
        obj = bpy.data.objects.get(wo.name)
        if obj:
            reset_ob(obj)


def reset_ob(ob):
    if not hasattr(bpy.context.scene, "wiggle"):
        return
    wo = bpy.context.scene.wiggle.list.get(ob.name)
    if wo:
        for wb in wo.list:
            if ob.pose and wb.name in ob.pose.bones:
                reset_bone(ob.pose.bones.get(wb.name))


def reset_bone(b):
    """Reset bone physics state to rest position."""
    mat_world = b.id_data.matrix_world
    
    # Tail position
    tail_world = mat_world @ Matrix.Translation(b.tail)
    b.wiggle.position = tail_world.translation.copy()
    b.wiggle.position_last = b.wiggle.position.copy()
    b.wiggle.velocity = ZERO_VEC.copy()
    b.wiggle.collision_normal = ZERO_VEC.copy()
    
    # Head position
    head_world = (mat_world @ b.matrix).translation
    b.wiggle.position_head = head_world.copy()
    b.wiggle.position_last_head = head_world.copy()
    b.wiggle.velocity_head = ZERO_VEC.copy()
    b.wiggle.collision_normal_head = ZERO_VEC.copy()
    
    # Matrix & Collision state
    b.wiggle.matrix = flatten(decompose_matrix_no_scale(mat_world @ b.matrix))
    b.wiggle.collision_point = ZERO_VEC.copy()
    b.wiggle.collision_point_head = ZERO_VEC.copy()
    b.wiggle.collision_ob = None
    b.wiggle.collision_ob_head = None


def build_list():
    if not hasattr(bpy.context.scene, "wiggle"):
        return
    bpy.context.scene.wiggle.list.clear()
    
    for ob in bpy.context.scene.objects:
        if ob.type != 'ARMATURE':
            continue
        
        wigglebones = []
        for b in ob.pose.bones:
            if b.wiggle_tail or (b.wiggle_head and not b.bone.use_connect):
                wigglebones.append(b)
                b.wiggle_enable = True
            else:
                b.wiggle_enable = False
        
        if not wigglebones:
            ob.wiggle_enable = False
            continue
        
        ob.wiggle_enable = True
        wo = bpy.context.scene.wiggle.list.add()
        wo.name = ob.name
        for b in wigglebones:
            wb = wo.list.add()
            wb.name = b.name


def update_prop(self, context, prop):
    if prop in ['wiggle_mute', 'wiggle_enable']:
        build_list()
    
    if isinstance(self, bpy.types.PoseBone) and context.selected_pose_bones:
        if len(context.selected_pose_bones) > 1:
            for b in context.selected_pose_bones:
                if b != self:
                    b[prop] = self[prop]
                    
        if prop in ['wiggle_head', 'wiggle_tail']:
            build_list()
            for b in context.selected_pose_bones:
                reset_bone(b)
                
    if context.scene.wiggle.is_rendering:
        context.scene.wiggle.is_rendering = False


def get_parent(b):
    p = b.parent
    if not p:
        return None
    if p.wiggle_enable and (not p.wiggle_mute) and (
        (p.wiggle_head and not p.bone.use_connect) or p.wiggle_tail
    ):
        return p
    return get_parent(p)


def length_world(b) -> float:
    """Get bone length in world space."""
    head = b.id_data.matrix_world @ b.head
    tail = b.id_data.matrix_world @ b.tail
    return (head - tail).length


# ------------------------------------------------------------------------
#    POLLS
# ------------------------------------------------------------------------

def collider_poll(self, object):
    return object.type == 'MESH'


def wind_poll(self, object):
    return object.field and object.field.type == 'WIND'


# ------------------------------------------------------------------------
#    PHYSICS HELPERS
# ------------------------------------------------------------------------

def bone_get_fac(mass1: float, mass2: float) -> float:
    if mass1 == mass2:
        return 0.5
    return mass1 / (mass1 + mass2)


def bone_spring(target: Vector, position: Vector, stiff: float, 
                dt2: float, iterations: int) -> Vector:
    s = target - position
    Fs = s * stiff / iterations
    displacement = Fs * dt2
    if displacement.length > s.length:
        return s
    return displacement


def bone_stretch(target: Vector, position: Vector, fac: float) -> Vector:
    return (target - position) * (1 - fac)


# ------------------------------------------------------------------------
#    COLLISION LOGIC (FIXED)
# ------------------------------------------------------------------------

def get_closest_point(pos: Vector, colliders: list, dg) -> tuple:
    """Find closest point on any collider mesh."""
    best_dist = float('inf')
    best_data = None

    for collider in colliders:
        try:
            collider_eval = collider.evaluated_get(dg)
        except:
            collider_eval = collider
        
        cmw = collider_eval.matrix_world
        try:
            cmw_inv = cmw.inverted()
        except ValueError:
            continue
            
        local_pos = cmw_inv @ pos
        
        try:
            result, p, n, face_index = collider_eval.closest_point_on_mesh(local_pos)
        except:
            continue
            
        if not result:
            continue
            
        world_p = cmw @ p
        dist = (world_p - pos).length
        
        if dist < best_dist:
            best_dist = dist
            # Transform normal to world space (rotation only)
            world_n = (cmw.to_3x3() @ n).normalized()
            best_data = (world_p, world_n, collider)
            
    return best_data, best_dist


def collide(b, dg, dt: float, head: bool = False):
    """
    Handle collision for bone head or tail.
    
    FIX: Clamps displacement to prevent explosion.
    """
    if head:
        pos = b.wiggle.position_head.copy()
        cp = b.wiggle.collision_point_head
        co = b.wiggle.collision_ob_head
        
        collider_type = b.wiggle_collider_type_head
        wiggle_collider = b.wiggle_collider_head
        wiggle_collection = b.wiggle_collider_collection_head
        
        radius = b.wiggle_radius_head
        sticky = b.wiggle_sticky_head
        friction = b.wiggle_friction_head
    else:
        pos = b.wiggle.position.copy()
        cp = b.wiggle.collision_point
        co = b.wiggle.collision_ob
        
        collider_type = b.wiggle_collider_type
        wiggle_collider = b.wiggle_collider
        wiggle_collection = b.wiggle_collider_collection
        
        radius = b.wiggle_radius
        sticky = b.wiggle_sticky
        friction = b.wiggle_friction
    
    # Gather colliders
    colliders = []
    if collider_type == 'Object' and wiggle_collider:
        if wiggle_collider.name in bpy.context.scene.objects:
            colliders = [wiggle_collider]
    elif collider_type == 'Collection' and wiggle_collection:
        if wiggle_collection in bpy.context.scene.collection.children_recursive:
            colliders = [ob for ob in wiggle_collection.objects if ob.type == 'MESH']
            
    if not colliders:
        return

    best_data, best_dist = get_closest_point(pos, colliders, dg)
    
    collision_occurred = False
    collision_normal = ZERO_VEC.copy()
    new_pos = pos.copy()

    if best_data:
        world_p, world_n, collider = best_data
        
        # Check if penetrating
        to_surface = world_p - pos
        is_penetrating = to_surface.length > 0.0001 and world_n.dot(to_surface.normalized()) > 0.01
        
        # Collision condition
        needs_collision = (
            is_penetrating or 
            (best_dist < radius) or 
            (co and best_dist < (radius + sticky))
        )
        
        if needs_collision:
            # Calculate target position (on surface + radius offset)
            target_pos = world_p + world_n * radius
            
            # --- FIX: Clamp the push distance ---
            push = target_pos - pos
            push = clamp_vector(push, MAX_PUSH)
            new_pos = pos + push
            
            # Apply friction if sticky collision
            if co:
                try:
                    collision_point_world = co.matrix_world @ cp
                    new_pos = new_pos.lerp(collision_point_world, friction * 0.5)
                except:
                    pass
            
            # --- FIX: Dampen velocity on collision ---
            if not head:
                vel = b.wiggle.velocity
                vn = vel.dot(world_n)
                if vn < 0:  # Moving into surface
                    # Remove penetrating component and dampen
                    b.wiggle.velocity = (vel - world_n * vn) * VELOCITY_DECAY_ON_COLLISION
            else:
                vel = b.wiggle.velocity_head
                vn = vel.dot(world_n)
                if vn < 0:
                    b.wiggle.velocity_head = (vel - world_n * vn) * VELOCITY_DECAY_ON_COLLISION

            collision_occurred = True
            co = collider
            try:
                cp = relative_matrix(
                    collider.matrix_world, 
                    Matrix.Translation(new_pos)
                ).translation
            except:
                cp = ZERO_VEC.copy()
            collision_normal = world_n.copy()

    if not collision_occurred:
        co = None
        collision_normal = ZERO_VEC.copy()

    # Validate before assignment
    new_pos = sanitize_vector(new_pos, pos)
    collision_normal = sanitize_vector(collision_normal)

    if head:
        b.wiggle.position_head = new_pos
        b.wiggle.collision_point_head = cp if collision_occurred else ZERO_VEC.copy()
        b.wiggle.collision_ob_head = co
        b.wiggle.collision_normal_head = collision_normal
    else:
        b.wiggle.position = new_pos
        b.wiggle.collision_point = cp if collision_occurred else ZERO_VEC.copy()
        b.wiggle.collision_ob = co
        b.wiggle.collision_normal = collision_normal


def collide_full_bone(b, dg):
    """
    Handle collision along entire bone length.
    
    FIX: Removed dangerous amplification, added push clamping.
    """
    pos_head = b.wiggle.position_head.copy()
    pos_tail = b.wiggle.position.copy()
    
    # Gather colliders from both head and tail settings
    def gather_colliders(ctype, cobj, ccol):
        found = []
        if ctype == 'Object' and cobj:
            if cobj.name in bpy.context.scene.objects:
                found = [cobj]
        elif ctype == 'Collection' and ccol:
            if ccol in bpy.context.scene.collection.children_recursive:
                found = [ob for ob in ccol.objects if ob.type == 'MESH']
        return found

    colliders = gather_colliders(
        b.wiggle_collider_type, b.wiggle_collider, b.wiggle_collider_collection
    )
    colliders += gather_colliders(
        b.wiggle_collider_type_head, b.wiggle_collider_head, b.wiggle_collider_collection_head
    )
    colliders = list(set(colliders))
    
    if not colliders:
        return

    full_settings = bpy.context.scene.wiggle.full_bone_collision
    steps = full_settings.steps
    
    radius = max(b.wiggle_radius, b.wiggle_radius_head, 0.02)
    
    bone_vec = pos_tail - pos_head
    bone_len = bone_vec.length
    if bone_len < 0.0001:
        return

    max_penetration = 0.0
    push_direction = ZERO_VEC.copy()

    # Sample points along bone
    for i in range(steps + 1):
        t = i / steps
        sample_pos = pos_head + (bone_vec * t)
        
        best_data, best_dist = get_closest_point(sample_pos, colliders, dg)
        
        if not best_data:
            continue
            
        world_p, world_n, collider = best_data
        
        to_surface = world_p - sample_pos
        dist = to_surface.length
        
        # Check if inside mesh
        is_inside = dist > 0.0001 and world_n.dot(to_surface.normalized()) > 0
        
        penetration = 0.0
        if is_inside:
            penetration = dist + radius
        elif dist < radius:
            penetration = radius - dist
        
        if penetration > max_penetration:
            max_penetration = penetration
            push_direction = world_n.copy()

    if max_penetration > 0:
        # --- FIX: Clamp push to safe maximum ---
        push_magnitude = min(max_penetration, MAX_PUSH)
        push_vector = push_direction * push_magnitude
        
        # Validate
        push_vector = sanitize_vector(push_vector)
        
        # Apply damped velocity
        if b.wiggle_tail:
            vn = b.wiggle.velocity.dot(push_direction)
            if vn < 0:
                b.wiggle.velocity = (
                    b.wiggle.velocity - push_direction * vn
                ) * VELOCITY_DECAY_ON_COLLISION

        if b.wiggle_head and not b.bone.use_connect:
            vn = b.wiggle.velocity_head.dot(push_direction)
            if vn < 0:
                b.wiggle.velocity_head = (
                    b.wiggle.velocity_head - push_direction * vn
                ) * VELOCITY_DECAY_ON_COLLISION

        # Apply push to both ends uniformly
        # --- FIX: Removed the dangerous 1/t amplification ---
        if b.wiggle_head and not b.bone.use_connect:
            b.wiggle.position_head = sanitize_vector(
                b.wiggle.position_head + push_vector, pos_head
            )
            b.wiggle.collision_normal_head = push_direction
            
        if b.wiggle_tail:
            b.wiggle.position = sanitize_vector(
                b.wiggle.position + push_vector, pos_tail
            )
            b.wiggle.collision_normal = push_direction


def update_matrix(b, last: bool = False):
    """
    Update bone's wiggle matrix from physics positions.
    
    FIX: Scale extraction prevents compounding.
    """
    id_world = b.id_data.matrix_world
    bone = b.bone
    bw = b.wiggle

    parent = get_parent(b)
    
    if parent:
        # --- FIX: Extract parent matrix WITHOUT inherited scale ---
        parent_wiggle_mat = unflatten(parent.wiggle.matrix)
        parent_wiggle_clean = decompose_matrix_no_scale(parent_wiggle_mat)
        
        real_mat = relative_matrix(parent.matrix, b.matrix)
        mat = parent_wiggle_clean @ real_mat
        
        # Always use clean (no-scale) for intermediate calculations
        m2 = decompose_matrix_no_scale(mat)
    else:
        mat = id_world @ b.matrix
        m2 = decompose_matrix_no_scale(mat)
    
    # Apply head position offset if enabled
    if b.wiggle_head and not bone.use_connect:
        head_offset = bw.position_head - m2.translation
        m2 = Matrix.Translation(head_offset) @ m2
        mat = m2
    
    # Calculate rotation from position
    tail_relative = bw.position - m2.translation
    if tail_relative.length > 0.0001:
        rxz = tail_relative.to_track_quat('Y', 'Z')
    else:
        rxz = Quaternion()
    rot = rxz.to_matrix().to_4x4()
    
    # Calculate scale (Y-axis stretch)
    l_world = length_world(b)
    if l_world < 0.0001:
        l_world = 0.0001
    
    if b.wiggle_head and not bone.use_connect:
        # Distance from head to tail in physics space
        physics_length = (bw.position_head - bw.position).length
    else:
        # Distance from bone origin to tail
        physics_length = (m2.translation - bw.position).length
    
    sy = physics_length / l_world
    
    # --- FIX: Tighter scale clamping ---
    sy = max(MIN_SCALE, min(sy, MAX_SCALE))
    
    scale = Matrix.Identity(4)
    scale[1][1] = sy
    
    # Store the matrix (without scale for chain calculations)
    # Apply scale only for final bone transform
    clean_matrix = decompose_matrix_no_scale(m2) @ rot
    bw.matrix = flatten(clean_matrix)
    
    if last:
        # Final matrix includes scale
        final_world = clean_matrix @ scale
        
        try:
            id_world_inv = id_world.inverted()
            b.matrix = id_world_inv @ final_world
        except ValueError:
            pass  # Singular matrix, skip assignment


def get_pin(b):
    for c in b.constraints:
        if c.type in ['DAMPED_TRACK', 'TRACK_TO', 'LOCKED_TRACK'] and c.target and not c.mute:
            return c
    return None


def pin(b):
    c = get_pin(b)
    if c:
        goal = c.target.matrix_world
        if c.subtarget:
            try:
                goal = goal @ c.target.pose.bones[c.subtarget].matrix
            except:
                pass
        b.wiggle.position = b.wiggle.position * (1 - c.influence) + goal.translation * c.influence


def move(b, dg, dt: float, dt2: float):
    """Apply forces and move bone positions."""
    enable_full_bone = bpy.context.scene.wiggle.full_bone_collision.enable_fullbone_collision
    
    if b.wiggle_tail:
        damp = max(min(1 - b.wiggle_damp * dt, 1), 0)
        
        vel = b.wiggle.velocity * damp
        vel = clamp_vector(vel, MAX_VELOCITY)
        b.wiggle.velocity = vel
        
        # Gravity
        F = bpy.context.scene.gravity * b.wiggle_gravity
        
        # Wind
        if b.wiggle_wind_ob:
            try:
                wind_dir = b.wiggle_wind_ob.matrix_world.to_3x3() @ Vector((0, 0, 1))
                fac = 1 - b.wiggle_wind_ob.field.wind_factor * abs(
                    wind_dir.dot((b.wiggle.position - Matrix(b.wiggle.matrix).translation).normalized())
                )
                F += wind_dir * fac * b.wiggle_wind_ob.field.strength * b.wiggle_wind / b.wiggle_mass
            except:
                pass
        
        # --- FIX: Clamp displacement ---
        displacement = vel + F * dt2
        displacement = clamp_vector(displacement, MAX_DISPLACEMENT)
        b.wiggle.position = sanitize_vector(b.wiggle.position + displacement, b.wiggle.position)
        
        pin(b)
        
        if enable_full_bone:
            collide_full_bone(b, dg)
        else:
            collide(b, dg, dt)
    
    if b.wiggle_head and not b.bone.use_connect:
        damp = max(min(1 - b.wiggle_damp_head * dt, 1), 0)
        
        vel = b.wiggle.velocity_head * damp
        vel = clamp_vector(vel, MAX_VELOCITY)
        b.wiggle.velocity_head = vel
        
        F = bpy.context.scene.gravity * b.wiggle_gravity_head
        
        if b.wiggle_wind_ob_head:
            try:
                wind_dir = b.wiggle_wind_ob_head.matrix_world.to_3x3() @ Vector((0, 0, 1))
                F += wind_dir * b.wiggle_wind_ob_head.field.strength * b.wiggle_wind_head / b.wiggle_mass_head
            except:
                pass
        
        displacement = vel + F * dt2
        displacement = clamp_vector(displacement, MAX_DISPLACEMENT)
        b.wiggle.position_head = sanitize_vector(b.wiggle.position_head + displacement, b.wiggle.position_head)
        
        if enable_full_bone:
            collide_full_bone(b, dg)
        else:
            collide(b, dg, dt, True)
    
    update_matrix(b)


def constrain(b, i: int, dg, dt: float, dt2: float, iterations: int):
    """Apply spring and stretch constraints."""
    bw = b.wiggle
    p = get_parent(b)
    
    if p:
        parent_mat = unflatten(p.wiggle.matrix)
        mat = parent_mat @ relative_matrix(p.matrix, b.matrix)
    else:
        mat = b.id_data.matrix_world @ b.matrix
    
    update_p = False
    enable_full_bone = bpy.context.scene.wiggle.full_bone_collision.enable_fullbone_collision

    # SPRING CONSTRAINTS
    if b.wiggle_head and not b.bone.use_connect:
        target = mat.translation
        s = bone_spring(target, bw.position_head, b.wiggle_stiff_head, dt2, iterations)
        
        if p and b.wiggle_chain_head:
            if p.wiggle_tail:
                fac = bone_get_fac(b.wiggle_mass_head, p.wiggle_mass) if i else p.wiggle_stretch
                p.wiggle.position -= s * fac
            else:
                fac = bone_get_fac(b.wiggle_mass_head, p.wiggle_mass_head)
                p.wiggle.position_head -= s * fac
            bw.position_head += s * (1 - fac)
        else:
            bw.position_head += s

        mat = Matrix.LocRotScale(bw.position_head, mat.decompose()[1], ONE_VEC)
        target = mat @ Vector((0, b.bone.length, 0))
        
        if b.wiggle_tail:
            s = bone_spring(target, bw.position, b.wiggle_stiff, dt2, iterations)
            if b.wiggle_chain:
                fac = bone_get_fac(b.wiggle_mass, b.wiggle_mass_head)
                bw.position_head -= s * fac
                bw.position += s * (1 - fac)
            else:
                bw.position += s
        else:
            bw.position = target
    else:
        mat = Matrix.LocRotScale(mat.decompose()[0], mat.decompose()[1], ONE_VEC)
        target = mat @ Vector((0, b.bone.length, 0))
        s = bone_spring(target, bw.position, b.wiggle_stiff, dt2, iterations)
        
        if p and b.wiggle_chain and p.wiggle_tail:
            fac = bone_get_fac(b.wiggle_mass, p.wiggle_mass)
            if get_pin(b):
                fac = 1 - b.wiggle_stretch
            if i == 0:
                fac = p.wiggle_stretch
            
            if p == b.parent and b.bone.use_connect:
                p.wiggle.position -= s * fac
            else:
                p_mat = unflatten(bw.matrix)
                tailpos = p_mat @ Vector((0, b.bone.length, 0))
                midpos = (p_mat.translation + tailpos) / 2
                v1 = midpos - unflatten(p.wiggle.matrix).translation
                tailpos -= s * fac
                midpos = (p_mat.translation + tailpos) / 2
                v2 = midpos - unflatten(p.wiggle.matrix).translation
                
                if v1.length > 0.0001:
                    sc = v2.length / v1.length
                    q = v1.rotation_difference(v2)
                    v3 = q @ (p.wiggle.position - unflatten(p.wiggle.matrix).translation)
                    p.wiggle.position = unflatten(p.wiggle.matrix).translation + v3 * sc
                
            bw.position += s * (1 - fac)
            update_p = True
        else:
            bw.position += s

    # STRETCH CONSTRAINTS
    if b.wiggle_head and not b.bone.use_connect:
        if p:
            if b.parent == p and p.wiggle_tail:
                offset = (b.id_data.matrix_world @ b.head - b.id_data.matrix_world @ p.tail).length
                direction = (bw.position_head - p.wiggle.position)
                if direction.length > 0.0001:
                    target = p.wiggle.position + direction.normalized() * offset
                else:
                    target = p.wiggle.position
            else:
                p_mat = unflatten(p.wiggle.matrix)
                targetpos = p_mat @ relative_matrix(p.matrix, b.parent.matrix) @ Vector((0, b.parent.length, 0))
                offset = (b.id_data.matrix_world @ b.head - b.id_data.matrix_world @ b.parent.tail).length
                direction = bw.position_head - targetpos
                if direction.length > 0.0001:
                    target = targetpos + direction.normalized() * offset
                else:
                    target = targetpos
        elif b.parent:
            ptail = b.id_data.matrix_world @ b.parent.tail
            offset = (b.id_data.matrix_world @ b.head - ptail).length
            direction = bw.position_head - ptail
            if direction.length > 0.0001:
                target = ptail + direction.normalized() * offset
            else:
                target = ptail
        else:
            target = mat.translation

        s = bone_stretch(target, bw.position_head, b.wiggle_stretch_head)
        
        if p and b.wiggle_chain_head:
            if p.wiggle_tail:
                fac = bone_get_fac(b.wiggle_mass_head, p.wiggle_mass) if i else p.wiggle_stretch
                p.wiggle.position -= s * fac
            else:
                fac = bone_get_fac(b.wiggle_mass_head, p.wiggle_mass_head) if i else p.wiggle_stretch_head
                p.wiggle.position_head -= s * fac
            bw.position_head += s * (1 - fac)
        else:
            bw.position_head += s

        direction = bw.position - bw.position_head
        if direction.length > 0.0001:
            target = bw.position_head + direction.normalized() * length_world(b)
        else:
            target = bw.position_head + Vector((0, length_world(b), 0))
            
        if b.wiggle_tail:
            s = bone_stretch(target, bw.position, b.wiggle_stretch)
            if b.wiggle_chain:
                fac = bone_get_fac(b.wiggle_mass, b.wiggle_mass_head) if i else b.wiggle_stretch_head
                bw.position_head -= s * fac
                bw.position += s * (1 - fac)
            else:
                bw.position += s
        else:
            bw.position = target

    else:
        direction = bw.position - mat.translation
        if direction.length > 0.0001:
            target = mat.translation + direction.normalized() * length_world(b)
        else:
            target = mat.translation + Vector((0, length_world(b), 0))
            
        s = bone_stretch(target, bw.position, b.wiggle_stretch)
        
        if p and b.wiggle_chain and p.wiggle_tail:
            fac = bone_get_fac(b.wiggle_mass, p.wiggle_mass)
            if get_pin(b):
                fac = 1 - b.wiggle_stretch
            if i == 0:
                fac = p.wiggle_stretch
                
            if p == b.parent and b.bone.use_connect:
                p.wiggle.position -= s * fac
            else:
                bw_mat = unflatten(bw.matrix)
                headpos = bw_mat.translation
                p_mat = unflatten(p.wiggle.matrix)
                v1 = headpos - p_mat.translation
                headpos -= s * fac
                v2 = headpos - p_mat.translation
                
                if v1.length > 0.0001:
                    sc = v2.length / v1.length
                    q = v1.rotation_difference(v2)
                    v3 = q @ (p.wiggle.position - p_mat.translation)
                    p.wiggle.position = p_mat.translation + v3 * sc
                
            bw.position += s * (1 - fac)
            update_p = True
        else:
            bw.position += s

    # Collision and matrix updates
    if update_p:
        if enable_full_bone:
            collide_full_bone(p, dg)
        else:
            collide(p, dg, dt)
        update_matrix(p)

    if b.wiggle_tail:
        if enable_full_bone:
            collide_full_bone(b, dg)
        else:
            collide(b, dg, dt)

    if b.wiggle_head:
        if enable_full_bone:
            collide_full_bone(b, dg)
        else:
            collide(b, dg, dt, True)

    update_matrix(b)


# ------------------------------------------------------------------------
#    HANDLERS
# ------------------------------------------------------------------------

@persistent
def wiggle_pre(scene):
    if (scene.wiggle.lastframe == scene.frame_current and not scene.wiggle.reset) or scene.wiggle.is_rendering:
        return

    if not scene.wiggle_enable:
        reset_scene()
        return

    for wo in scene.wiggle.list:
        ob = scene.objects.get(wo.name)
        if not ob:
            build_list()
            return

        if getattr(ob, "wiggle_mute", False) or getattr(ob, "wiggle_freeze", False):
            reset_ob(ob)
            continue

        ob_pose_bones = getattr(ob.pose, "bones", {})

        for wb in wo.list:
            b = ob_pose_bones.get(wb.name)
            if not b:
                build_list()
                return
            if getattr(b, "wiggle_mute", False) or not (
                getattr(b, "wiggle_head", False) or getattr(b, "wiggle_tail", False)
            ):
                reset_bone(b)
                continue

            bw = b.wiggle
            if not bw.collision_col:
                if b.wiggle_collider_collection or b.wiggle_collider_collection_head:
                    bw.collision_col = scene.collection
                elif b.wiggle_collider or b.wiggle_collider_head:
                    bw.collision_col = scene.collection

            b.location = ZERO_VEC.copy()
            b.rotation_quaternion = Quaternion((1, 0, 0, 0))
            b.rotation_euler = ZERO_VEC.copy()
            b.scale = ONE_VEC.copy()

    if bpy.context.view_layer:
        bpy.context.view_layer.update()


@persistent
def wiggle_post(scene, dg):
    wiggle = scene.wiggle

    if (wiggle.lastframe == scene.frame_current) and not wiggle.reset:
        return
    if not scene.wiggle_enable or wiggle.is_rendering:
        return

    lastframe = wiggle.lastframe
    frame_start = scene.frame_start
    frame_end = scene.frame_end
    frame_current = scene.frame_current
    frame_is_preroll = wiggle.is_preroll
    frame_loop = wiggle.loop

    if (frame_current == frame_start) and not frame_loop and not frame_is_preroll:
        bpy.ops.wiggle.reset()
        return

    if frame_current >= lastframe:
        frames_elapsed = frame_current - lastframe
    else:
        e1 = (frame_end - lastframe) + (frame_current - frame_start) + 1
        e2 = lastframe - frame_current
        frames_elapsed = min(e1, e2)

    if frames_elapsed > 4 or frame_is_preroll:
        frames_elapsed = 1

    wiggle.lastframe = frame_current

    fps = scene.render.fps if scene.render.fps > 0 else 24
    dt = 1.0 / fps
    dt2 = dt * dt

    wiggle_list = wiggle.list
    wiggle_iterations = wiggle.iterations
    objects = scene.objects

    # Main Simulation Loop
    for _ in range(frames_elapsed):
        for wo in wiggle_list:
            ob = objects.get(wo.name)
            if not ob:
                continue

            if getattr(ob, 'wiggle_mute', False) or getattr(ob, 'wiggle_freeze', False):
                continue

            pose_bones = ob.pose.bones

            bones = []
            for wb in wo.list:
                if wb.name in pose_bones:
                    pb = pose_bones[wb.name]
                    if not getattr(pb, 'wiggle_mute', False) and (
                        getattr(pb, 'wiggle_head', False) or getattr(pb, 'wiggle_tail', False)
                    ):
                        bones.append(pb)

            for b in bones:
                b.wiggle.collision_normal = ZERO_VEC.copy()
                b.wiggle.collision_normal_head = ZERO_VEC.copy()
                move(b, dg, dt, dt2)

            for i in range(wiggle_iterations):
                idx = wiggle_iterations - 1 - i
                for b in bones:
                    constrain(b, idx, dg, dt, dt2, wiggle_iterations)

            for b in bones:
                update_matrix(b, True)

            # --- FIX: Velocity calculation with displacement clamping ---
            for b in bones:
                bw = b.wiggle

                # Tail velocity
                pos_delta = bw.position - bw.position_last
                pos_delta = clamp_vector(pos_delta, MAX_DISPLACEMENT)
                
                vb = ZERO_VEC.copy()
                if bw.collision_normal.length > 0.0001:
                    reflected = bw.velocity.reflect(bw.collision_normal)
                    vb = reflected.project(bw.collision_normal) * b.wiggle_bounce
                
                new_vel = pos_delta + vb
                bw.velocity = clamp_vector(sanitize_vector(new_vel), MAX_VELOCITY)

                # Head velocity
                pos_delta_head = bw.position_head - bw.position_last_head
                pos_delta_head = clamp_vector(pos_delta_head, MAX_DISPLACEMENT)
                
                vb_head = ZERO_VEC.copy()
                if bw.collision_normal_head.length > 0.0001:
                    reflected = bw.velocity_head.reflect(bw.collision_normal_head)
                    vb_head = reflected.project(bw.collision_normal_head) * b.wiggle_bounce_head
                
                new_vel_head = pos_delta_head + vb_head
                bw.velocity_head = clamp_vector(sanitize_vector(new_vel_head), MAX_VELOCITY)

                bw.position_last = bw.position.copy()
                bw.position_last_head = bw.position_head.copy()


@persistent
def wiggle_render_pre(scene):
    if hasattr(scene, "wiggle"):
        scene.wiggle.is_rendering = True


@persistent
def wiggle_render_post(scene):
    if hasattr(scene, "wiggle"):
        scene.wiggle.is_rendering = False


@persistent
def wiggle_render_cancel(scene):
    if hasattr(scene, "wiggle"):
        scene.wiggle.is_rendering = False


@persistent
def wiggle_load(dummy):
    build_list()
    if bpy.context.scene and hasattr(bpy.context.scene, "wiggle"):
        bpy.context.scene.wiggle.is_rendering = False


# ------------------------------------------------------------------------
#    OPERATORS
# ------------------------------------------------------------------------

class WiggleCopy(bpy.types.Operator):
    """Copy active wiggle settings to selected bones"""
    bl_idname = "wiggle.copy"
    bl_label = "Copy Settings to Selected"

    @classmethod
    def poll(cls, context):
        return (
            context.mode == 'POSE' and 
            context.active_pose_bone and 
            len(context.selected_pose_bones) > 1
        )

    def execute(self, context):
        b = context.active_pose_bone
        props = [
            'wiggle_mute', 'wiggle_head', 'wiggle_tail',
            'wiggle_head_mute', 'wiggle_tail_mute',
            'wiggle_mass', 'wiggle_stiff', 'wiggle_stretch', 'wiggle_damp',
            'wiggle_gravity', 'wiggle_wind_ob', 'wiggle_wind',
            'wiggle_collider_type', 'wiggle_collider', 'wiggle_collider_collection',
            'wiggle_radius', 'wiggle_friction', 'wiggle_bounce', 'wiggle_sticky',
            'wiggle_chain',
            'wiggle_mass_head', 'wiggle_stiff_head', 'wiggle_stretch_head',
            'wiggle_damp_head', 'wiggle_gravity_head', 'wiggle_wind_ob_head',
            'wiggle_wind_head', 'wiggle_collider_type_head', 'wiggle_collider_head',
            'wiggle_collider_collection_head', 'wiggle_radius_head',
            'wiggle_friction_head', 'wiggle_bounce_head', 'wiggle_sticky_head',
            'wiggle_chain_head'
        ]
        
        for ob in context.selected_pose_bones:
            if ob != b:
                for prop in props:
                    setattr(ob, prop, getattr(b, prop))
        
        return {'FINISHED'}


class WiggleReset(bpy.types.Operator):
    """Reset scene wiggle physics to rest state"""
    bl_idname = "wiggle.reset"
    bl_label = "Reset Physics"

    @classmethod
    def poll(cls, context):
        return context.scene.wiggle_enable and context.mode in ['OBJECT', 'POSE']

    def execute(self, context):
        context.scene.wiggle.reset = True
        context.scene.frame_set(context.scene.frame_current)
        context.scene.wiggle.reset = False
        
        rebuild = False
        for wo in context.scene.wiggle.list:
            ob = context.scene.objects.get(wo.name)
            if not ob:
                rebuild = True
                continue
            for wb in wo.list:
                if ob.pose and wb.name in ob.pose.bones:
                    b = ob.pose.bones.get(wb.name)
                    reset_bone(b)
                else:
                    rebuild = True
        
        context.scene.wiggle.lastframe = context.scene.frame_current
        if rebuild:
            build_list()
        
        return {'FINISHED'}


class WiggleSelect(bpy.types.Operator):
    """Select wiggle bones on selected objects in pose mode"""
    bl_idname = "wiggle.select"
    bl_label = "Select Enabled"

    @classmethod
    def poll(cls, context):
        return context.mode == 'POSE'

    def execute(self, context):
        bpy.ops.pose.select_all(action='DESELECT')
        
        rebuild = False
        for wo in context.scene.wiggle.list:
            ob = context.scene.objects.get(wo.name)
            if not ob:
                rebuild = True
                continue
            for wb in wo.list:
                if ob.pose and wb.name in ob.pose.bones:
                    b = ob.pose.bones.get(wb.name)
                    b.bone.select = True
                else:
                    rebuild = True
        
        if rebuild:
            build_list()
        
        return {'FINISHED'}


class WiggleBake(bpy.types.Operator):
    """Bake this object's visible wiggle bones to keyframes"""
    bl_idname = "wiggle.bake"
    bl_label = "Bake Wiggle"

    @classmethod
    def poll(cls, context):
        return context.object and context.object.type == 'ARMATURE'

    def execute(self, context):
        obj = context.object
        scene = context.scene

        def push_nla():
            if scene.wiggle.bake_overwrite:
                return
            if not scene.wiggle.bake_nla:
                return
            if not obj.animation_data:
                return
            if not obj.animation_data.action:
                return
            action = obj.animation_data.action
            track = obj.animation_data.nla_tracks.new()
            track.name = action.name
            track.strips.new(action.name, int(action.frame_range[0]), action)

        push_nla()
        bpy.ops.wiggle.reset()

        # Preroll
        duration = scene.frame_end - scene.frame_start
        preroll = scene.wiggle.preroll
        scene.wiggle.is_preroll = False
        bpy.ops.wiggle.select()
        bpy.ops.wiggle.reset()

        while preroll >= 0:
            if scene.wiggle.loop and duration > 0:
                frame = scene.frame_end - (preroll % duration)
                scene.frame_set(frame)
            else:
                scene.frame_set(scene.frame_start)
            scene.wiggle.is_preroll = True
            preroll -= 1

        # Clear ID props
        for bone in obj.pose.bones:
            if "wiggle" in bone:
                del bone["wiggle"]

        # Bake
        with context.temp_override(selected_objects=[obj], selected_pose_bones=context.selected_pose_bones):
            bpy.ops.nla.bake(
                frame_start=scene.frame_start,
                frame_end=scene.frame_end,
                only_selected=True,
                visual_keying=True,
                use_current_action=scene.wiggle.bake_overwrite,
                bake_types={'POSE'},
                channel_types={'LOCATION', 'ROTATION', 'SCALE'}
            )

        scene.wiggle.is_preroll = False
        obj.wiggle_freeze = True
        
        if not scene.wiggle.bake_overwrite and obj.animation_data and obj.animation_data.action:
            obj.animation_data.action.name = 'WiggleAction'

        return {'FINISHED'}


# ------------------------------------------------------------------------
#    UI PANELS
# ------------------------------------------------------------------------

class WigglePanel:
    bl_category = 'Animation'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'

    @classmethod
    def poll(cls, context):
        return context.object


class WIGGLE_PT_Settings(WigglePanel, bpy.types.Panel):
    bl_label = 'Wiggle 2'
    bl_idname = "WIGGLE_PT_Settings"

    def draw(self, context):
        row = self.layout.row()
        icon = 'HIDE_ON' if not context.scene.wiggle_enable else 'SCENE_DATA'
        row.prop(context.scene, "wiggle_enable", icon=icon, text="", emboss=False)
        
        if not context.scene.wiggle_enable:
            row.label(text='Scene muted.')
            return
        if context.object.type != 'ARMATURE':
            row.label(text=' Select armature.')
            return

        if context.object.wiggle_freeze:
            row.prop(context.object, 'wiggle_freeze', icon='FREEZE', icon_only=True, emboss=False)
            row.label(text='Wiggle Frozen after Bake.')
            return
        
        icon = 'HIDE_ON' if context.object.wiggle_mute else 'ARMATURE_DATA'
        row.prop(context.object, 'wiggle_mute', icon=icon, icon_only=True, invert_checkbox=True, emboss=False)
        
        if context.object.wiggle_mute:
            row.label(text='Armature muted.')
            return
        if not context.active_pose_bone:
            row.label(text=' Select pose bone.')
            return

        icon = 'HIDE_ON' if context.active_pose_bone.wiggle_mute else 'BONE_DATA'
        row.prop(context.active_pose_bone, 'wiggle_mute', icon=icon, icon_only=True, invert_checkbox=True, emboss=False)
        
        if context.active_pose_bone.wiggle_mute:
            row.label(text='Bone muted.')


class WIGGLE_PT_Head(WigglePanel, bpy.types.Panel):
    bl_label = 'Head Physics'
    bl_parent_id = 'WIGGLE_PT_Settings'
    bl_options = {'HEADER_LAYOUT_EXPAND'}

    @classmethod
    def poll(cls, context):
        return (
            context.scene.wiggle_enable and
            context.object and not context.object.wiggle_mute and
            context.active_pose_bone and not context.active_pose_bone.wiggle_mute and
            not context.active_pose_bone.bone.use_connect
        )

    def draw_header(self, context):
        self.layout.prop(context.active_pose_bone, 'wiggle_head', text="")

    def draw(self, context):
        b = context.active_pose_bone
        if not b.wiggle_head:
            return

        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        col = layout.column(align=True)
        col.prop(b, 'wiggle_mass_head')
        col.prop(b, 'wiggle_stiff_head')
        col.prop(b, 'wiggle_stretch_head')
        col.prop(b, 'wiggle_damp_head')
        col.separator()
        col.prop(b, 'wiggle_gravity_head')
        
        row = col.row(align=True)
        row.prop(b, 'wiggle_wind_ob_head')
        sub = row.row(align=True)
        sub.ui_units_x = 5
        sub.prop(b, 'wiggle_wind_head', text='')
        
        col.separator()
        col.prop(b, 'wiggle_collider_type_head', text='Collisions')

        collision = False
        if b.wiggle_collider_type_head == 'Object':
            row = col.row(align=True)
            row.prop_search(b, 'wiggle_collider_head', context.scene, 'objects', text=' ')
            collision = bool(b.wiggle_collider_head)
        else:
            row = col.row(align=True)
            row.prop_search(b, 'wiggle_collider_collection_head', bpy.data, 'collections', text=' ')
            collision = bool(b.wiggle_collider_collection_head)

        if collision:
            col = layout.column(align=True)
            col.prop(b, 'wiggle_radius_head')
            col.prop(b, 'wiggle_friction_head')
            col.prop(b, 'wiggle_bounce_head')
            col.prop(b, 'wiggle_sticky_head')
        
        layout.prop(b, 'wiggle_chain_head')


class WIGGLE_PT_Tail(WigglePanel, bpy.types.Panel):
    bl_label = 'Tail Physics'
    bl_parent_id = 'WIGGLE_PT_Settings'
    bl_options = {'HEADER_LAYOUT_EXPAND'}

    @classmethod
    def poll(cls, context):
        return (
            context.scene.wiggle_enable and
            context.object and not context.object.wiggle_mute and
            context.active_pose_bone and not context.active_pose_bone.wiggle_mute
        )

    def draw_header(self, context):
        self.layout.prop(context.active_pose_bone, 'wiggle_tail', text="")

    def draw(self, context):
        b = context.active_pose_bone
        if not b.wiggle_tail:
            return

        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        col = layout.column(align=True)
        col.prop(b, 'wiggle_mass')
        col.prop(b, 'wiggle_stiff')
        col.prop(b, 'wiggle_stretch')
        col.prop(b, 'wiggle_damp')
        col.separator()
        col.prop(b, 'wiggle_gravity')
        
        row = col.row(align=True)
        row.prop(b, 'wiggle_wind_ob')
        sub = row.row(align=True)
        sub.ui_units_x = 5
        sub.prop(b, 'wiggle_wind', text='')
        
        col.separator()
        col.prop(b, 'wiggle_collider_type', text='Collisions')

        collision = False
        if b.wiggle_collider_type == 'Object':
            row = col.row(align=True)
            row.prop_search(b, 'wiggle_collider', context.scene, 'objects', text=' ')
            collision = bool(b.wiggle_collider)
        else:
            row = col.row(align=True)
            row.prop_search(b, 'wiggle_collider_collection', bpy.data, 'collections', text=' ')
            collision = bool(b.wiggle_collider_collection)

        if collision:
            col = layout.column(align=True)
            col.prop(b, 'wiggle_radius')
            col.prop(b, 'wiggle_friction')
            col.prop(b, 'wiggle_bounce')
            col.prop(b, 'wiggle_sticky')
        
        layout.prop(b, 'wiggle_chain')


class WIGGLE_PT_Utilities(WigglePanel, bpy.types.Panel):
    bl_label = 'Global Wiggle Utilities'
    bl_idname = "WIGGLE_PT_Utilities"
    bl_parent_id = 'WIGGLE_PT_Settings'
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return context.scene.wiggle_enable

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False
        
        col = layout.column(align=True)
        if context.object and context.object.wiggle_enable and context.mode == 'POSE':
            col.operator('wiggle.copy')
            col.operator('wiggle.select')
        col.operator('wiggle.reset')
        
        layout.prop(context.scene.wiggle, 'loop')
        layout.prop(context.scene.wiggle, 'iterations')


class WIGGLE_PT_Fullbone_Collision(bpy.types.Panel):
    bl_label = "Full Bone Collision Settings"
    bl_idname = "WIGGLE_PT_Fullbone_Collision"
    bl_parent_id = "WIGGLE_PT_Utilities"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return context.scene.wiggle_enable

    def draw(self, context):
        layout = self.layout
        fullbone = context.scene.wiggle.full_bone_collision

        layout.prop(fullbone, "enable_fullbone_collision")
        if fullbone.enable_fullbone_collision:
            layout.prop(fullbone, "steps")
            layout.prop(fullbone, "collision_threshold")
            layout.prop(fullbone, "dot_threshold")


class WIGGLE_PT_Bake(WigglePanel, bpy.types.Panel):
    bl_label = 'Bake Wiggle'
    bl_idname = "WIGGLE_PT_Bake"
    bl_parent_id = 'WIGGLE_PT_Utilities'
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return (
            context.scene.wiggle_enable and
            context.object and context.object.wiggle_enable and
            context.mode == 'POSE'
        )

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False
        
        layout.prop(context.scene.wiggle, 'preroll')
        layout.prop(context.scene.wiggle, 'bake_overwrite')
        
        row = layout.row()
        row.enabled = not context.scene.wiggle.bake_overwrite
        row.prop(context.scene.wiggle, 'bake_nla')
        
        layout.operator('wiggle.bake')


# ------------------------------------------------------------------------
#    PROPERTY CLASSES
# ------------------------------------------------------------------------

class WiggleBoneItem(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(override={'LIBRARY_OVERRIDABLE'})


class WiggleItem(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(override={'LIBRARY_OVERRIDABLE'})
    list: bpy.props.CollectionProperty(
        type=WiggleBoneItem, 
        override={'LIBRARY_OVERRIDABLE', 'USE_INSERTION'}
    )


class WiggleBone(bpy.types.PropertyGroup):
    matrix: bpy.props.FloatVectorProperty(
        name='Matrix', size=16, subtype='MATRIX', 
        override={'LIBRARY_OVERRIDABLE'}
    )
    position: bpy.props.FloatVectorProperty(
        subtype='TRANSLATION', 
        override={'LIBRARY_OVERRIDABLE'}
    )
    position_last: bpy.props.FloatVectorProperty(
        subtype='TRANSLATION', 
        override={'LIBRARY_OVERRIDABLE'}
    )
    velocity: bpy.props.FloatVectorProperty(
        subtype='VELOCITY', 
        override={'LIBRARY_OVERRIDABLE'}
    )
    
    collision_point: bpy.props.FloatVectorProperty(
        subtype='TRANSLATION', 
        override={'LIBRARY_OVERRIDABLE'}
    )
    collision_ob: bpy.props.PointerProperty(
        type=bpy.types.Object, 
        override={'LIBRARY_OVERRIDABLE'}
    )
    collision_normal: bpy.props.FloatVectorProperty(
        subtype='TRANSLATION', 
        override={'LIBRARY_OVERRIDABLE'}
    )
    collision_col: bpy.props.PointerProperty(
        type=bpy.types.Collection, 
        override={'LIBRARY_OVERRIDABLE'}
    )
    
    position_head: bpy.props.FloatVectorProperty(
        subtype='TRANSLATION', 
        override={'LIBRARY_OVERRIDABLE'}
    )
    position_last_head: bpy.props.FloatVectorProperty(
        subtype='TRANSLATION', 
        override={'LIBRARY_OVERRIDABLE'}
    )
    velocity_head: bpy.props.FloatVectorProperty(
        subtype='VELOCITY', 
        override={'LIBRARY_OVERRIDABLE'}
    )
    
    collision_point_head: bpy.props.FloatVectorProperty(
        subtype='TRANSLATION', 
        override={'LIBRARY_OVERRIDABLE'}
    )
    collision_ob_head: bpy.props.PointerProperty(
        type=bpy.types.Object, 
        override={'LIBRARY_OVERRIDABLE'}
    )
    collision_normal_head: bpy.props.FloatVectorProperty(
        subtype='TRANSLATION', 
        override={'LIBRARY_OVERRIDABLE'}
    )


class FullBoneCollisionSettings(bpy.types.PropertyGroup):
    enable_fullbone_collision: bpy.props.BoolProperty(
        name='Enable Full Bone Collision',
        description='Enable collision detection for the entire wiggle bone',
        default=False
    )
    steps: bpy.props.IntProperty(
        name='Steps',
        description='Interpolation steps for collision detection along bone',
        min=1, default=10, soft_max=64, max=100
    )
    collision_threshold: bpy.props.FloatProperty(
        name='Collision Threshold',
        description='Minimum movement distance to consider a collision',
        min=0.001, default=0.02, precision=4
    )
    dot_threshold: bpy.props.FloatProperty(
        name='Dot Threshold',
        description='Sensitivity threshold for dot product check',
        min=0.01, max=1.0, default=0.1, precision=2
    )


class WiggleObject(bpy.types.PropertyGroup):
    list: bpy.props.CollectionProperty(
        type=WiggleItem, 
        override={'LIBRARY_OVERRIDABLE'}
    )


class WiggleScene(bpy.types.PropertyGroup):
    lastframe: bpy.props.IntProperty()
    iterations: bpy.props.IntProperty(
        name='Quality',
        description='Constraint solver iterations for chain physics',
        min=1, default=2, soft_max=10, max=100
    )
    loop: bpy.props.BoolProperty(
        name='Loop Physics',
        description='Physics continues as timeline loops',
        default=True
    )
    list: bpy.props.CollectionProperty(
        type=WiggleItem, 
        override={'LIBRARY_OVERRIDABLE', 'USE_INSERTION'}
    )
    preroll: bpy.props.IntProperty(
        name='Preroll',
        description='Frames to run simulation before bake',
        min=0, default=0
    )
    is_preroll: bpy.props.BoolProperty(default=False)
    bake_overwrite: bpy.props.BoolProperty(
        name='Overwrite Current Action',
        description='Bake into current action instead of creating new',
        default=False
    )
    bake_nla: bpy.props.BoolProperty(
        name='Current Action to NLA',
        description='Move existing animation into NLA strip',
        default=False
    )
    is_rendering: bpy.props.BoolProperty(default=False)
    reset: bpy.props.BoolProperty(default=False)
    full_bone_collision: bpy.props.PointerProperty(type=FullBoneCollisionSettings)


# ------------------------------------------------------------------------
#    REGISTRATION
# ------------------------------------------------------------------------

classes = (
    WiggleBoneItem,
    WiggleItem,
    WiggleBone,
    FullBoneCollisionSettings,
    WiggleObject,
    WiggleScene,
    WiggleReset,
    WiggleCopy,
    WiggleSelect,
    WiggleBake,
    WIGGLE_PT_Settings,
    WIGGLE_PT_Head,
    WIGGLE_PT_Tail,
    WIGGLE_PT_Utilities,
    WIGGLE_PT_Fullbone_Collision,
    WIGGLE_PT_Bake,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    # Property group assignments
    bpy.types.PoseBone.wiggle = bpy.props.PointerProperty(
        type=WiggleBone, 
        override={'LIBRARY_OVERRIDABLE'}
    )
    bpy.types.Object.wiggle = bpy.props.PointerProperty(
        type=WiggleObject, 
        override={'LIBRARY_OVERRIDABLE'}
    )
    bpy.types.Scene.wiggle = bpy.props.PointerProperty(
        type=WiggleScene, 
        override={'LIBRARY_OVERRIDABLE'}
    )

    # Scene/Object/Bone toggles
    bpy.types.Scene.wiggle_enable = bpy.props.BoolProperty(
        name='Enable Scene',
        description='Enable wiggle on this scene',
        default=False,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_enable')
    )
    bpy.types.Object.wiggle_enable = bpy.props.BoolProperty(
        name='Enable Armature',
        default=False,
        options={'HIDDEN'},
        override={'LIBRARY_OVERRIDABLE'}
    )
    bpy.types.Object.wiggle_mute = bpy.props.BoolProperty(
        name='Mute Armature',
        default=False,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_mute')
    )
    bpy.types.Object.wiggle_freeze = bpy.props.BoolProperty(
        name='Freeze Wiggle',
        default=False,
        override={'LIBRARY_OVERRIDABLE'}
    )
    bpy.types.PoseBone.wiggle_enable = bpy.props.BoolProperty(
        name='Enable Bone',
        default=False,
        options={'HIDDEN'},
        override={'LIBRARY_OVERRIDABLE'}
    )
    bpy.types.PoseBone.wiggle_mute = bpy.props.BoolProperty(
        name='Mute Bone',
        default=False,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_mute')
    )
    bpy.types.PoseBone.wiggle_head = bpy.props.BoolProperty(
        name='Bone Head',
        default=False,
        override={'LIBRARY_OVERRIDABLE'},
        options={'HIDDEN'},
        update=lambda s, c: update_prop(s, c, 'wiggle_head')
    )
    bpy.types.PoseBone.wiggle_tail = bpy.props.BoolProperty(
        name='Bone Tail',
        default=False,
        override={'LIBRARY_OVERRIDABLE'},
        options={'HIDDEN'},
        update=lambda s, c: update_prop(s, c, 'wiggle_tail')
    )
    bpy.types.PoseBone.wiggle_head_mute = bpy.props.BoolProperty(
        name='Bone Head Mute',
        default=False,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_head_mute')
    )
    bpy.types.PoseBone.wiggle_tail_mute = bpy.props.BoolProperty(
        name='Bone Tail Mute',
        default=False,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_tail_mute')
    )

    # Tail physics properties
    bpy.types.PoseBone.wiggle_mass = bpy.props.FloatProperty(
        name='Mass', min=0.01, default=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_mass')
    )
    bpy.types.PoseBone.wiggle_stiff = bpy.props.FloatProperty(
        name='Stiff', min=0, default=400,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_stiff')
    )
    bpy.types.PoseBone.wiggle_stretch = bpy.props.FloatProperty(
        name='Stretch', min=0, default=0, max=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_stretch')
    )
    bpy.types.PoseBone.wiggle_damp = bpy.props.FloatProperty(
        name='Damp', min=0, default=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_damp')
    )
    bpy.types.PoseBone.wiggle_gravity = bpy.props.FloatProperty(
        name='Gravity', default=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_gravity')
    )
    bpy.types.PoseBone.wiggle_wind_ob = bpy.props.PointerProperty(
        name='Wind', type=bpy.types.Object, poll=wind_poll,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_wind_ob')
    )
    bpy.types.PoseBone.wiggle_wind = bpy.props.FloatProperty(
        name='Wind Multiplier', default=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_wind')
    )
    bpy.types.PoseBone.wiggle_chain = bpy.props.BoolProperty(
        name='Chain', default=True,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_chain')
    )

    # Head physics properties
    bpy.types.PoseBone.wiggle_mass_head = bpy.props.FloatProperty(
        name='Mass', min=0.01, default=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_mass_head')
    )
    bpy.types.PoseBone.wiggle_stiff_head = bpy.props.FloatProperty(
        name='Stiff', min=0, default=400,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_stiff_head')
    )
    bpy.types.PoseBone.wiggle_stretch_head = bpy.props.FloatProperty(
        name='Stretch', min=0, default=0, max=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_stretch_head')
    )
    bpy.types.PoseBone.wiggle_damp_head = bpy.props.FloatProperty(
        name='Damp', min=0, default=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_damp_head')
    )
    bpy.types.PoseBone.wiggle_gravity_head = bpy.props.FloatProperty(
        name='Gravity', default=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_gravity_head')
    )
    bpy.types.PoseBone.wiggle_wind_ob_head = bpy.props.PointerProperty(
        name='Wind', type=bpy.types.Object, poll=wind_poll,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_wind_ob_head')
    )
    bpy.types.PoseBone.wiggle_wind_head = bpy.props.FloatProperty(
        name='Wind', default=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_wind_head')
    )
    bpy.types.PoseBone.wiggle_chain_head = bpy.props.BoolProperty(
        name='Chain', default=True,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_chain_head')
    )

    # Tail collision properties
    bpy.types.PoseBone.wiggle_collider_type = bpy.props.EnumProperty(
        name='Collider Type',
        items=[
            ('Object', 'Object', 'Collide with selected mesh'),
            ('Collection', 'Collection', 'Collide with all meshes in collection')
        ],
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_collider_type')
    )
    bpy.types.PoseBone.wiggle_collider = bpy.props.PointerProperty(
        name='Collider Object', type=bpy.types.Object, poll=collider_poll,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_collider')
    )
    bpy.types.PoseBone.wiggle_collider_collection = bpy.props.PointerProperty(
        name='Collider Collection', type=bpy.types.Collection,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_collider_collection')
    )
    bpy.types.PoseBone.wiggle_radius = bpy.props.FloatProperty(
        name='Radius', min=0, default=0.05,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_radius')
    )
    bpy.types.PoseBone.wiggle_friction = bpy.props.FloatProperty(
        name='Friction', min=0, default=0.5, soft_max=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_friction')
    )
    bpy.types.PoseBone.wiggle_bounce = bpy.props.FloatProperty(
        name='Bounce', min=0, default=0.2, soft_max=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_bounce')
    )
    bpy.types.PoseBone.wiggle_sticky = bpy.props.FloatProperty(
        name='Sticky', min=0, default=0, soft_max=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_sticky')
    )

    # Head collision properties
    bpy.types.PoseBone.wiggle_collider_type_head = bpy.props.EnumProperty(
        name='Collider Type',
        items=[
            ('Object', 'Object', 'Collide with selected mesh'),
            ('Collection', 'Collection', 'Collide with all meshes in collection')
        ],
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_collider_type_head')
    )
    bpy.types.PoseBone.wiggle_collider_head = bpy.props.PointerProperty(
        name='Collider Object', type=bpy.types.Object, poll=collider_poll,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_collider_head')
    )
    bpy.types.PoseBone.wiggle_collider_collection_head = bpy.props.PointerProperty(
        name='Collider Collection', type=bpy.types.Collection,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_collider_collection_head')
    )
    bpy.types.PoseBone.wiggle_radius_head = bpy.props.FloatProperty(
        name='Radius', min=0, default=0.05,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_radius_head')
    )
    bpy.types.PoseBone.wiggle_friction_head = bpy.props.FloatProperty(
        name='Friction', min=0, default=0.5, soft_max=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_friction_head')
    )
    bpy.types.PoseBone.wiggle_bounce_head = bpy.props.FloatProperty(
        name='Bounce', min=0, default=0.2, soft_max=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_bounce_head')
    )
    bpy.types.PoseBone.wiggle_sticky_head = bpy.props.FloatProperty(
        name='Sticky', min=0, default=0, soft_max=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_sticky_head')
    )

    # Handlers
    bpy.app.handlers.frame_change_pre.append(wiggle_pre)
    bpy.app.handlers.frame_change_post.append(wiggle_post)
    bpy.app.handlers.render_pre.append(wiggle_render_pre)
    bpy.app.handlers.render_post.append(wiggle_render_post)
    bpy.app.handlers.render_cancel.append(wiggle_render_cancel)
    bpy.app.handlers.load_post.append(wiggle_load)


def unregister():
    # Remove handlers
    handlers = [
        (bpy.app.handlers.frame_change_pre, wiggle_pre),
        (bpy.app.handlers.frame_change_post, wiggle_post),
        (bpy.app.handlers.render_pre, wiggle_render_pre),
        (bpy.app.handlers.render_post, wiggle_render_post),
        (bpy.app.handlers.render_cancel, wiggle_render_cancel),
        (bpy.app.handlers.load_post, wiggle_load),
    ]
    for handler_list, handler in handlers:
        if handler in handler_list:
            handler_list.remove(handler)

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
