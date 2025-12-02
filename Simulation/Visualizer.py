"""
Test file for inspecting models
--------------------
"""

import os

import pydot
from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer
from pydrake.systems.analysis import Simulator  

# ------------------ Settings ------------------
visualize = False  # True = only visualize, False = run full simulation
meshcat = StartMeshcat()
# Adjust the path to where the URDF is in your directory
model_path=os.path.join("..","SDF_file","project_07_object_handover.sdf")

# ------------------ Functions ------------------
def create_sim_scene(sim_time_step):
    """Creates a MultibodyPlant + SceneGraph diagram."""
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    
    Parser(plant).AddModelsFromUrl("file://" + os.path.abspath(model_path))
    plant.Finalize()
    AddDefaultVisualization(builder, meshcat)
    return builder.Build()

def run_visualizer():
    """Minimal visualization using ModelVisualizer."""
    visualizer = ModelVisualizer(meshcat=meshcat)
    visualizer.parser().AddModelsFromUrl("file://" + os.path.abspath(model_path))
    visualizer.Run()

def run_simulation(sim_time_step=0.01):
    """Run full simulation if visualize=False."""
    diagram = create_sim_scene(sim_time_step)
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()
    simulator.set_publish_every_time_step(True)
    sim_time = 8.0  # seconds
    simulator.AdvanceTo(sim_time)

'''# Save diagram imageu
    svg_data = diagram.GetGraphvizString(max_depth=2)
    graph = pydot.graph_from_dot_data(svg_data)[0]
    graph.write_png("figures/block_diagram_01.png")
    print("Block diagram saved as figures/block_diagram_01.png")'''

# ------------------ Helper: diagnostics for why ../models may be untracked ------------------
def git_models_diagnostics():
    sim_dir = os.path.dirname(__file__)
    candidate = os.path.normpath(os.path.join(sim_dir, "..", "models"))
    print("Diagnostics for models folder:")
    print("  Simulation script dir:", sim_dir)
    print("  Candidate models path:", candidate)
    print("  Exists?:", os.path.exists(candidate))
    if os.path.exists(candidate):
        print("  Is directory?:", os.path.isdir(candidate))
        try:
            files = os.listdir(candidate)
            print("  Contains (sample up to 10):", files[:10])
        except Exception as e:
            print("  Error listing files:", str(e))
    print("")
    print("If the directory exists but is untracked, run from repo root:")
    print("  git add models")
    print("Or from Simulation/ if it's ../models:")
    print("  git add ../models")
    print("If it's empty, create a .gitkeep and add it.")
    print("To see if files are ignored:")
    print("  git check-ignore -v <path_to_file>")
    print("To force-add an ignored file:")
    print("  git add -f <path_to_file>")

# ------------------ Main ------------------
if visualize:
    run_visualizer()
else:
    run_simulation()
