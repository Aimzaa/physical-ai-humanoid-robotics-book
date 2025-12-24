# Data Model: Physical AI & Humanoid Robotics Book

## 1. Entity Overview

This document defines the data model for the Physical AI & Humanoid Robotics book, which will be implemented as a Docusaurus documentation website. The model encompasses the content structure, navigation elements, and metadata required for the book.

## 2. Core Entities

### 2.1 Book
- **name**: string (required) - The title of the book
- **description**: string (required) - Brief description of the book's purpose
- **version**: string (required) - Version identifier following semantic versioning
- **authors**: array of strings - List of book authors/contributors
- **created_date**: date (required) - Date the book was created
- **last_updated**: date (required) - Date of last content update
- **word_count**: integer - Total word count across all chapters
- **modules**: array of Module - List of modules that compose the book

### 2.2 Module
- **id**: string (required) - Unique identifier for the module (e.g., "module-1-robotic-nervous-system")
- **title**: string (required) - Display title of the module
- **description**: string (required) - Brief description of the module content
- **goal**: string (required) - The main goal of the module
- **outcomes**: array of strings (required) - Learning outcomes for the module
- **scope**: string (required) - What is included in the module
- **non_goals**: string (required) - What is excluded from the module
- **deliverables**: array of strings (required) - Tangible outputs of the module
- **chapters**: array of Chapter (required) - List of chapters in the module
- **order**: integer (required) - Sequential order of the module in the book

### 2.3 Chapter
- **id**: string (required) - Unique identifier for the chapter (e.g., "chapter-1-1-introduction-ros2")
- **title**: string (required) - Display title of the chapter
- **goal**: string (required) - The main goal of the chapter
- **learning_objectives**: array of strings (required) - Specific learning objectives
- **overview**: string (required) - Brief overview of chapter content
- **key_concepts**: array of strings (required) - Main theoretical concepts covered
- **step_by_step_breakdown**: string (required) - Detailed implementation steps
- **code_examples**: array of CodeExample - Practical code examples in the chapter
- **diagrams**: array of Diagram - Visual diagrams and illustrations
- **case_study**: string (optional) - Real-world application example
- **references**: array of Reference (required) - Academic and practical references
- **review_questions**: array of string (required) - Questions to test understanding
- **practical_exercises**: array of string (required) - Hands-on exercises for students
- **module_id**: string (required) - Reference to parent module
- **order**: integer (required) - Sequential order of the chapter in the module
- **word_count**: integer - Word count for the chapter (500-1200)

### 2.4 CodeExample
- **id**: string (required) - Unique identifier for the code example
- **language**: string (required) - Programming language (e.g., "python", "bash", "yaml")
- **code**: string (required) - The actual code content
- **description**: string (required) - Explanation of what the code does
- **file_path**: string (optional) - Path to the actual file if external
- **chapter_id**: string (required) - Reference to parent chapter

### 2.5 Diagram
- **id**: string (required) - Unique identifier for the diagram
- **title**: string (required) - Title of the diagram
- **description**: string (required) - Description of what the diagram illustrates
- **file_path**: string (required) - Path to the image file in /static/img/
- **alt_text**: string (required) - Alternative text for accessibility
- **chapter_id**: string (required) - Reference to parent chapter

### 2.6 Reference
- **id**: string (required) - Unique identifier for the reference
- **title**: string (required) - Title of the referenced work
- **url**: string (optional) - URL if it's a web reference
- **author**: string (optional) - Author of the reference
- **publication**: string (optional) - Publication name or source
- **year**: integer (optional) - Year of publication
- **type**: string (required) - Type of reference (e.g., "web", "academic", "documentation")
- **chapter_id**: string (required) - Reference to parent chapter

## 3. Relationships

### 3.1 Book-Module Relationship
- A Book has many Modules (1 to many)
- Each Module belongs to exactly one Book
- The relationship is mandatory for all Modules

### 3.2 Module-Chapter Relationship
- A Module has many Chapters (1 to many)
- Each Chapter belongs to exactly one Module
- The relationship is mandatory for all Chapters

### 3.3 Chapter-CodeExample Relationship
- A Chapter has many CodeExamples (1 to many)
- Each CodeExample belongs to exactly one Chapter
- The relationship is optional (some chapters may not have code examples)

### 3.4 Chapter-Diagram Relationship
- A Chapter has many Diagrams (1 to many)
- Each Diagram belongs to exactly one Chapter
- The relationship is optional (some chapters may not have diagrams)

### 3.5 Chapter-Reference Relationship
- A Chapter has many References (1 to many)
- Each Reference belongs to exactly one Chapter
- The relationship is mandatory for all Chapters (minimum 1 reference required)

## 4. Validation Rules

### 4.1 Book Validation
- word_count must be between 6,000 and 10,000
- modules array must contain exactly 4 modules
- modules must be ordered sequentially (1-4)

### 4.2 Module Validation
- order must be between 1 and 4
- outcomes array must contain 5-7 items
- chapters array must contain 3-6 chapters
- chapters must be ordered sequentially (1-6)

### 4.3 Chapter Validation
- word_count must be between 500 and 1,200
- learning_objectives must contain 3-5 items
- key_concepts must contain at least 2 items
- references array must contain at least 1 item
- review_questions must contain at least 3 items
- practical_exercises must contain at least 1 item

### 4.4 CodeExample Validation
- language must be a valid programming language identifier
- code must not be empty
- description must be provided

### 4.5 Diagram Validation
- file_path must exist in the /static/img/ directory
- alt_text must be provided for accessibility
- title must be descriptive

### 4.6 Reference Validation
- If type is "web", URL must be provided and valid
- If type is "academic", author and publication must be provided
- If type is "documentation", URL must be provided

## 5. State Transitions

### 5.1 Chapter States
- **draft**: Initial state when chapter is created
- **in_review**: When chapter is being reviewed
- **approved**: When chapter has passed review
- **published**: When chapter is included in the deployed book
- **archived**: When chapter is no longer part of the book

### 5.2 Module States
- **draft**: When module is being created
- **in_progress**: When chapters are being added/edited
- **complete**: When all chapters are approved
- **published**: When module is included in the deployed book

## 6. Navigation Structure

### 6.1 Sidebar Organization
The sidebar will be organized hierarchically:
- Book Title (top level)
  - Module 1 Title (collapsible)
    - Chapter 1.1
    - Chapter 1.2
    - ...
  - Module 2 Title (collapsible)
    - Chapter 2.1
    - Chapter 2.2
    - ...
  - Module 3 Title (collapsible)
  - Module 4 Title (collapsible)
  - Capstone Project

### 6.2 Breadcrumb Navigation
Each page will include breadcrumb navigation showing the path from Book → Module → Chapter to help users understand their location in the content hierarchy.

## 7. Metadata Requirements

### 7.1 SEO Metadata
- **title**: Page-specific title for search engines
- **description**: Brief description of page content
- **keywords**: Relevant keywords for search
- **author**: Author of the specific page
- **date**: Last updated date

### 7.2 Accessibility Metadata
- **lang**: Language of the content (en for English)
- **dir**: Text direction (ltr for left-to-right)
- **role**: ARIA roles for screen readers
- **aria-label**: Accessible labels for navigation elements