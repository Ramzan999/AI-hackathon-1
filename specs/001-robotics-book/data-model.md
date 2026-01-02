# Data Model: Physical AI & Humanoid Robotics Book

## Entities

### Module
- **name**: string (e.g., "ROS 2 Nervous System", "Simulation", "NVIDIA Isaac Brain", "VLA Systems")
- **description**: string - Brief overview of the module content
- **pages**: array of Page entities - List of pages in the module
- **learningObjectives**: array of strings - What students should learn
- **prerequisites**: array of strings - Required knowledge before starting

### Page
- **title**: string - Page title displayed in navigation
- **slug**: string - URL-friendly identifier
- **type**: enum (Concepts, Hands-on Lab, Troubleshooting) - Page category
- **content**: string - Markdown content of the page
- **codeExamples**: array of CodeExample entities - Code snippets in the page
- **assets**: array of Asset entities - Images, videos, or other media

### CodeExample
- **language**: string (python, cpp, bash, etc.) - Programming language
- **code**: string - The actual code snippet
- **description**: string - Explanation of what the code does
- **fileName**: string - Suggested file name for the example
- **requirements**: array of strings - Dependencies needed to run

### Asset
- **type**: enum (image, video, document, interactive) - Asset category
- **path**: string - Relative path from static directory
- **altText**: string - Accessibility text for images
- **caption**: string - Description shown with the asset

### CapstoneProject
- **title**: string - Project name
- **description**: string - Overview of the project goals
- **requirements**: array of strings - Technical requirements
- **steps**: array of strings - Implementation phases
- **evaluationCriteria**: array of strings - How success will be measured
- **integrationPoints**: array of strings - How it connects to course modules

### DeploymentConfig
- **platform**: string (github-pages) - Hosting platform
- **branch**: string (gh-pages) - Deployment branch
- **buildScript**: string (npm run build) - Command to build site
- **outputDir**: string (build) - Directory to deploy
- **domain**: string - Custom domain if applicable

## Relationships

- Module **contains** many Page
- Page **contains** many CodeExample
- Page **contains** many Asset
- CapstoneProject **integrates** with many Module
- CodeExample **belongs to** one Page
- Asset **belongs to** one or many Page

## Validation Rules

- Module name must be unique within the course
- Page slug must be unique across all modules
- CodeExample language must be supported by Docusaurus syntax highlighting
- Asset path must exist in static directory
- Module must have at least one page of each type (Concepts, Hands-on Lab, Troubleshooting)
- CapstoneProject must reference valid modules

## State Transitions

Not applicable for static documentation site - all entities are created during development and published as static content.