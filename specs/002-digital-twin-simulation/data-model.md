# Data Model: Digital Twin Simulation Module

## Educational Content Structure

### Module Entity
- **Name**: String (required) - The module title
- **Description**: String (required) - Overview of the module content
- **Target Audience**: String (required) - Who the module is designed for
- **Focus Areas**: Array<String> (required) - Key topics covered in the module
- **Chapters**: Array<Chapter> (required) - List of chapters in the module
- **Status**: Enum (draft, reviewed, published) - Current state of the module

### Chapter Entity
- **Title**: String (required) - The chapter title
- **Content**: String (required) - The main content in MD format
- **Learning Objectives**: Array<String> (required) - What students should learn
- **Topics**: Array<String> (required) - Specific topics covered in the chapter
- **Simulation Examples**: Array<SimulationExample> (optional) - Associated simulation projects
- **Module ID**: String (required) - Reference to parent module

### Simulation Example Entity
- **Type**: Enum (gazebo, unity, sensor) (required) - Type of simulation
- **Description**: String (required) - Explanation of the simulation example
- **Configuration Files**: Array<String> (required) - Associated config files
- **ROS Integration**: String (required) - How it connects to ROS 2
- **Chapter ID**: String (required) - Reference to parent chapter
- **Status**: Enum (untested, tested, verified) - Implementation status

## Relationships
- Module (1) -> Chapter (many): One module contains multiple chapters
- Chapter (1) -> Simulation Example (many): One chapter can have multiple simulation examples

## Validation Rules
- Module name must be unique across all modules
- Module description must be at least 20 characters
- Each module must have 1-10 chapters
- Each chapter must have a unique title within its module
- Simulation examples must have valid configuration files
- All content must be in MD format as required by constitution

## State Transitions
- Draft -> Reviewed: Content has been reviewed by subject matter expert
- Reviewed -> Published: Content has been approved for public consumption
- Published -> Archived: Content is no longer current or relevant