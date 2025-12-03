"""
Test file for inspecting models
--------------------
"""

import os
import numpy as np
import pydot # For saving block diagram images

# Drake dependencies imports
from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer
from pydrake.systems.analysis import Simulator  

# ------------------ Settings ------------------
visualize = False  # True = only visualize, False = run full simulation
meshcat = StartMeshcat()

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
    sim_time = 3.0  # seconds

    # To rerun the simulation in Meshcat if needed
    meshcat.StartRecording()
    simulator.AdvanceTo(sim_time)
    meshcat.PublishRecording()

    # Saving simulation block diagram as PNG
    svg_data = diagram.GetGraphvizString(max_depth=2)
    graph = pydot.graph_from_dot_data(svg_data)[0]
    graph.write_png("figures/block_diagram_sim.png")
    print("Block diagram saved as figures/block_diagram_sim.png")

# ------------------ Main ------------------
if visualize:
    run_visualizer()
else:
    run_simulation()
