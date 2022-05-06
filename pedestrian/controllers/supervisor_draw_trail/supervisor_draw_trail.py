from controller import Supervisor

MAXIMUM_NUMBER_OF_COORDINATES = 10000
REFRESH_FACTOR = 10

# Create the trail shape with the correct number of coordinates.

class TrailController(Supervisor):
  def __init__(self):
    super(TrailController, self).__init__()
    # Set the refresh rate of this controller, and so, set the refresh rate of the line set.
    self.time_step = int(self.getBasicTimeStep())
    self.time_step *= REFRESH_FACTOR 
    self.target_node = self.getFromDef("TARGET")
   
  
    # Create the TRAIL Shape which will contain the green line set.
    self.create_trail_shape() 

    # Get interesting references to the TRAIL subnodes.
    self.trail_line_set_node = self.getFromDef("TRAIL_LINE_SET")
    self.coordinates_node = self.trail_line_set_node.getField("coord").getSFNode()
    self.point_field = self.coordinates_node.getField("point")
    self.coord_index_field = self.trail_line_set_node.getField("coordIndex")

    self.index = 0 # This points to the current position to be drawn.
    self.first_step = True # Only equals to true during the first step.


  def create_trail_shape(self):
    # If TRAIL exists in the world then silently remove it.
    existing_trail = self.getFromDef("TRAIL")
    if existing_trail:
      existing_trail.remove()

    trail_string = []

    # Create the TRAIL Shape.
    trail_string.append("DEF TRAIL Shape {\n")
    trail_string.append("  appearance Appearance {\n")
    trail_string.append("    material Material {\n")
    trail_string.append("      diffuseColor 0 1 0\n")
    trail_string.append("      emissiveColor 0 1 0\n")
    trail_string.append("    }\n")
    trail_string.append("  }\n")
    trail_string.append("  geometry DEF TRAIL_LINE_SET IndexedLineSet {\n")
    trail_string.append("    coord Coordinate {\n")
    trail_string.append("      point [\n")
    for _ in range(MAXIMUM_NUMBER_OF_COORDINATES):
      trail_string.append("      0 0 0\n") 
    trail_string.append("      ]\n") 
    trail_string.append("    }\n") 
    trail_string.append("    coordIndex [\n") 
    for _ in range(MAXIMUM_NUMBER_OF_COORDINATES):
        trail_string.append("      0 0 -1\n") 
    trail_string.append("    ]\n") 
    trail_string.append("  }\n") 
    trail_string.append("}\n") 

    trail_string = "".join(trail_string)

    # Import TRAIL and append it as the world root nodes.
    root = self.getRoot()
    root_children_field = root.getField("children")
    root_children_field.importMFNodeFromString(-1, trail_string)
    

  def run(self):
    while self.step(self.time_step) != -1:
      target_translation = self.target_node.getPosition()

      self.point_field.setMFVec3f(self.index, target_translation)

      # Update the line set indices.
      if self.index > 0: 
        # Link successive indices.
        self.coord_index_field.setMFInt32(3 * (self.index - 1), self.index - 1)
        self.coord_index_field.setMFInt32(3 * (self.index - 1) + 1, self.index)
      elif self.index == 0 and self.first_step == False:
        # Link the first and the last indices.
        self.coord_index_field.setMFInt32(3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1), 0)
        self.coord_index_field.setMFInt32(3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1) + 1, MAXIMUM_NUMBER_OF_COORDINATES - 1)

      # Unset the next indices.
      self.coord_index_field.setMFInt32(3 * self.index, self.index)
      self.coord_index_field.setMFInt32(3 * self.index + 1, self.index)

      # Update global variables.
      self.first_step = False
      self.index += 1
      self.index = self.index % MAXIMUM_NUMBER_OF_COORDINATES 
      
# main Python program
trail_controller = TrailController()
trail_controller.run()