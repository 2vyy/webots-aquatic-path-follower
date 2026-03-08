import sys
import os

def check_mesh_info(filepath):
    """
    Checks basic mesh information like face count and bounding box.
    """
    print(f"--- Checking {filepath} ---")
    try:
        import pymeshlab
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(filepath)
        mesh = ms.current_mesh()
        
        print(f"Faces: {mesh.face_number()}")
        print(f"Vertices: {mesh.vertex_number()}")
        
        bbox = mesh.bounding_box()
        print(f"Extents: X={bbox.dim_x():.3f}, Y={bbox.dim_y():.3f}, Z={bbox.dim_z():.3f}")
        
    except ImportError:
        print("pymeshlab not installed. Install via `pip install pymeshlab`.")
    except Exception as e:
        print(f"Error checking mesh: {e}")

def decimate_mesh(input_file, output_file, target_faces=50000):
    """
    1. Loads the original CAD export (e.g. blueboat.stl from Blueboat website which has 11,246,096 triangles).
    2. Checks if model is in millimeters (extents > 100), if so, scales by 0.001 to meters.
    3. Centers the model at the local origin so it spawns at its mass center in Webots.
    4. Decimates the mesh using Quadric Edge Collapse strategy safely down to `target_faces` (50,000).
    5. Saves as a Webots-friendly .obj file.
    """
    print(f"\n--- Decimating {input_file} to {target_faces} faces ---")
    
    try:
        import pymeshlab
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(input_file)
        
        bbox = ms.current_mesh().bounding_box()
        extents = [bbox.dim_x(), bbox.dim_y(), bbox.dim_z()]
        print(f"Original Extents: {extents}")
        print(f"Original Faces: {ms.current_mesh().face_number()}")
        
        # Scale to meters if the STL was exported in millimeters.
        if max(extents) > 100:
            print("Model is in mm. Scaling uniform 0.001 to meters...")
            ms.apply_filter("compute_matrix_from_scaling_or_normalization", axisx=0.001, axisy=0.001, axisz=0.001)
            
            # Recalculate bounding box after scale to center the boat
            bbox = ms.current_mesh().bounding_box()
            b_min = bbox.min()
            b_max = bbox.max()
            cx = (b_min[0] + b_max[0]) / 2
            cy = (b_min[1] + b_max[1]) / 2
            cz = (b_min[2] + b_max[2]) / 2
            print(f"Centering translation vector: [{-cx}, {-cy}, {-cz}]")
            ms.apply_filter("compute_matrix_from_translation", axisx=-cx, axisy=-cy, axisz=-cz)
            print("Scale and Center complete.")
        
        print(f"Applying Quadric Edge Collapse decimation to {target_faces} faces...")
        ms.apply_filter("meshing_decimation_quadric_edge_collapse", targetfacenum=target_faces, preservenormal=True)
        
        print(f"Final Faces: {ms.current_mesh().face_number()}")
        print(f"Saving optimized mesh to {output_file}...")
        ms.save_current_mesh(output_file)
        print("Success!")
        
    except ImportError:
        print("pymeshlab not installed. Install via `pip install pymeshlab`.")
    except Exception as e:
        print(f"Error decimating mesh: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--check":
        check_mesh_info("src/usv_description/meshes/blueboat.stl")
        check_mesh_info("src/usv_description/meshes/blueboat_visual.obj")
    else:
        print("Run `python3 process_mesh.py --check` to read the metadata of the existing mesh files.")
        print("Running the full decimation pipeline...")
        # Note: requires the massive raw 11.2M triangle blueboat.stl from Blueboat website
        decimate_mesh("src/usv_description/meshes/blueboat.stl", "src/usv_description/meshes/blueboat_visual.obj", target_faces=50000)
