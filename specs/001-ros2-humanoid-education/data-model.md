# Data Model: ROS2 Humanoid Education Module

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
- **Content**: String (required) - The main content in MD/MDX format
- **Learning Objectives**: Array<String> (required) - What students should learn
- **Topics**: Array<String> (required) - Specific topics covered in the chapter
- **Code Examples**: Array<CodeExample> (optional) - Associated code examples
- **Module ID**: String (required) - Reference to parent module

### Code Example Entity
- **Language**: String (required) - Programming language (e.g., Python, C++)
- **Code**: String (required) - The actual code content
- **Description**: String (required) - Explanation of what the code does
- **Usage Context**: String (required) - Where and how to use the example
- **Chapter ID**: String (required) - Reference to parent chapter
- **Test Status**: Enum (untested, tested, verified) - Execution status

## Relationships
- Module (1) -> Chapter (many): One module contains multiple chapters
- Chapter (1) -> Code Example (many): One chapter can have multiple code examples

## Validation Rules
- Module name must be unique across all modules
- Module description must be at least 20 characters
- Each module must have 1-10 chapters
- Each chapter must have a unique title within its module
- Code examples must have valid syntax for their specified language
- All content must be in MD/MDX format as required by constitution

## State Transitions
- Draft -> Reviewed: Content has been reviewed by subject matter expert
- Reviewed -> Published: Content has been approved for public consumption
- Published -> Archived: Content is no longer current or relevant