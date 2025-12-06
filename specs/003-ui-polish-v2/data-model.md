# Data Model: UI Polish V2

**Feature**: UI Polish and Structure Update (V2)
**Context**: Frontend Components

## Component Data Structures

### 1. Learning Journey Matrix (`LearningJourneyMatrix.js`)

**Description**: Data structure defining the content for the matrix cards.

```javascript
const matrixData = [
  {
    module: 1,
    title: "Foundations",
    description: "Master the physical AI architecture.",
    link: "/docs/module1/architecture-concepts",
    highlight: "Gold" // CSS class suffix
  },
  {
    module: 2,
    title: "Simulation",
    description: "Build robust Gazebo worlds.",
    link: "/docs/module2/gazebo-setup",
    highlight: "Gold"
  },
  {
    module: 3,
    title: "Perception",
    description: "Implement vSLAM and Nav2.",
    link: "/docs/module3/isaac-sim-intro",
    highlight: "Gold"
  },
  {
    module: 4,
    title: "Cognition",
    description: "Integrate LLMs and VLA models.",
    link: "/docs/module4/conversational-robotics",
    highlight: "Gold"
  }
];
```

### 2. Footer Navigation (`Footer/index.js`)

**Description**: Navigation links structure.

```javascript
const footerLinks = [
  {
    title: "Module 1: Foundations",
    to: "/docs/module1/architecture-concepts"
  },
  {
    title: "Module 2: Simulation",
    to: "/docs/module2/gazebo-setup"
  },
  {
    title: "Module 3: Perception",
    to: "/docs/module3/isaac-sim-intro"
  },
  {
    title: "Module 4: Cognition",
    to: "/docs/module4/conversational-robotics"
  }
];
```

## CSS Variables (`custom.css`)

**Description**: Global theme tokens.

| Variable | Value (Example) | Usage |
| :--- | :--- | :--- |
| `--ifm-font-family-base` | `'Inter', sans-serif` | Body text |
| `--ifm-heading-font-family` | `'Montserrat', sans-serif` | Headers |
| `--color-gold-primary` | `#D4AF37` | Highlights, Buttons |
| `--color-black-primary` | `#121212` | Backgrounds |
