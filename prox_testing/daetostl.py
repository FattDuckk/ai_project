import bpy
import os

# --- Configure Paths ---
# Absolute path to the folder containing your .dae files
input_folder = "/home/matt/ai_project/prox_testing/uarm_description/meshes/" # ADJUST THIS
# Absolute path to the folder where you want to save .stl files (can be the same)
output_folder = "/home/matt/ai_project/prox_testing/uarm_description/meshes/" # ADJUST THIS
# ---

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Get list of all .dae files in the input folder
dae_files = [f for f in os.listdir(input_folder) if f.lower().endswith(".dae")]

if not dae_files:
    print(f"No .dae files found in {input_folder}")
else:
    print(f"Found .dae files: {dae_files}")

for filename in dae_files:
    input_path = os.path.join(input_folder, filename)
    output_filename = os.path.splitext(filename)[0] + ".stl" # Change extension
    output_path = os.path.join(output_folder, output_filename)

    print(f"Processing: {filename} -> {output_filename}")

    # Clear existing scene
    bpy.ops.wm.read_factory_settings(use_empty=True)

    try:
        # Import the .dae file
        bpy.ops.wm.collada_import(filepath=input_path)

        # Select all imported objects (important for export)
        for obj in bpy.context.scene.objects:
            obj.select_set(True)
        bpy.context.view_layer.objects.active = bpy.context.scene.objects[0] # Set one active

        # Export as .stl
        # Check Blender documentation for available options if needed
        # (e.g., use_selection=True, ascii=False)
        bpy.ops.export_mesh.stl(filepath=output_path, use_selection=True, ascii=False) # Export only selected

        print(f"  Successfully exported to {output_path}")

    except Exception as e:
        print(f"  ERROR processing {filename}: {e}")

print("Batch conversion finished.")