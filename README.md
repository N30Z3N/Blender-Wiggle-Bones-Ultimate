**DISCLAIMER**:: I'm not author / owner of code, I simply used Gemini 3.0 pro to update this addon for New blender Version.
***

# Wiggle 2 (Ultimate Update) for Blender

**Spring-like physics for bone transforms, now with Full Bone Collisions and Blender 5.0+ Compatibility.**

![Blender Version](https://img.shields.io/badge/Blender-4.0%20%7C%205.0-orange?logo=blender)
![License](https://img.shields.io/badge/License-GPL%20v3-blue)

## 📖 Overview

**Wiggle 2** is a Blender add-on that adds procedural secondary motion to your rigs. It allows bones to simulate drag, jiggle, and follow-through automatically based on your animation, without requiring complex rigid body setups.

This **"Ultimate"** version combines the codebase of the original **Steve Miller** release with the advanced collision features of the **Labhatorian** fork. It has been extensively refactored to support **Blender 4.x and 5.0**, featuring optimized math, fixed NLA baking contexts, and a unified UI.

### ✨ Key Features
*   **Physics for Bones:** Add lag, bounce, and stretch to bone Tails (Rotation) and Heads (Translation).
*   **Full Bone Collision:** Unlike the original, this version can collide along the entire length of the bone, not just the tip.
*   **Wind & Gravity:** Reacts to standard Blender Force Fields.
*   **Looping:** Seamlessly loops physics for walk cycles and turntables.
*   **Non-Destructive:** Works on top of your existing keyframes.
*   **Baking:** Bake the simulation to keyframes or NLA strips for export to Game Engines.

---

## 📥 Installation

1.  Download the `wiggle_2_ultimate.py` file from this repository.
2.  Open Blender.
3.  Go to **Edit > Preferences > Add-ons**.
4.  Click **Install...** and select the downloaded file.
5.  Check the box next to **Animation: Wiggle 2 (Ultimate Update)** to enable it.

---

## 🚀 Quick Start

1.  **Select your Armature** in the 3D Viewport.
2.  Open the **N-Panel** (press `N`) and click the **Animation** tab.
3.  Under the **Wiggle 2** panel, check **Enable Scene**.
4.  Switch to **Pose Mode**.
5.  Select a bone you want to jiggle (e.g., a ponytail, cape, or belly).
6.  In the panel, check **Bone Tail** (for rotation physics) or **Bone Head** (for translation/bouncing).
7.  **Press Play** on the timeline. The bone will now react to movement!

---

## 🎛️ Settings Guide

### Bone Tail vs. Bone Head
*   **Bone Tail:** Rotates the bone to simulate the tip dragging behind. Best for tails, hair, cloth, and ears.
*   **Bone Head:** Translates the bone (moves it in space). Best for breast physics, belly jiggle, or loose accessories.

### Physics Properties
*   **Mass:** How heavy the object feels. Higher mass = slower reaction to forces.
*   **Stiffness:** How strongly the bone tries to return to its original pose.
    *   *High:* Vibrates fast, stiff wire.
    *   *Low:* Loose rope, slow oscillation.
*   **Damp:** Air resistance.
    *   *Low:* Swings forever.
    *   *High:* Stops moving quickly (like moving through water).
*   **Gravity:** Multiplier for the scene's gravity.
*   **Stretch:** Allows the bone to lengthen (rubber hose effect).

### Collisions (Standard)
*   **Collider Type:** Choose to collide with a specific **Object** or a whole **Collection**.
*   **Radius:** The size of the collision sphere at the bone tip.
*   **Bounce:** How much energy is preserved on impact (0 = thud, 1 = super ball).
*   **Friction:** How much the bone slides across the surface.

### 💥 Full Bone Collision (New Feature)
Located in the **Global Wiggle Utilities** panel.

Standard collision only checks if the *tip* of the bone hits something. **Full Bone Collision** checks multiple points along the length of the bone.

1.  Expand **Full Bone Collision Settings**.
2.  Check **Enable Full Bone Collision**.
3.  **Steps:** How many points along the bone to check (default 10). Higher = more accurate but slower.
4.  **Collision Threshold:** The margin of error for detecting a hit.

> **⚠️ Note:** Full Bone Collision is computationally expensive. Use standard collisions for simple objects and Full Collision only when necessary (e.g., long hair resting on shoulders).

---

## 🍞 Baking (Exporting)

Wiggle is a live simulation. To export to Unity, Unreal, or render farms, you must **Bake**.

1.  Go to the **Bake Wiggle** sub-panel.
2.  **Preroll:** How many frames to simulate *before* frame 1 (useful if the character starts in motion).
3.  **Overwrite Current Action:** If checked, writes keys to the active action. If unchecked, creates a new Action.
4.  **Current Action to NLA:** Pushes your original animation to the NLA stack so the wiggle acts as a layer on top.
5.  Click **Bake Wiggle**.

*The add-on creates standard Keyframes on the bones.*

---

## 🛠️ Troubleshooting

**"The simulation is glitching/exploding!"**
*   Your **Stiffness** might be too high for the **Mass**. Try increasing steps (Scene Properties) or lowering stiffness.
*   Ensure your armature Scale is applied (`Ctrl+A` > Scale).

**"Bake failed with Context Incorrect"**
*   This version fixes the context errors found in Blender 3.2+. Ensure you are using this updated version of the script.

**"My loop isn't looping"**
*   Check **Loop Physics** in the Utilities panel.
*   Ensure the start and end poses of your base animation are identical.

---

## 📜 Credits

*   **Original Author:** Steve Miller (shteeve3d)
*   **Fork Author (Collisions):** Labhatorian
*   **2024/2025 Refactor:** Updated for modern Blender API, optimized math, and unified features.

## 📄 License

This project is licensed under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html).
