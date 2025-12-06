# Data Model / Component Props

**Feature**: Docusaurus Home Page UI Overhaul

## UI Components

### HeroSection (`frontend/src/components/HeroSection.js`)

**Description**: The main landing area containing the title, subtitle, call to action, and 3D robot image.

| Prop Name | Type | Required | Description |
| :--- | :--- | :--- | :--- |
| `title` | string | Yes | Main headline (e.g., "Physical AI") |
| `subtitle` | string | Yes | Supporting text (e.g., "Humanoid Robotics...") |
| `ctaText` | string | Yes | Text for the primary button (e.g., "Start Learning") |
| `ctaLink` | string | Yes | URL for the CTA button |
| `imageSrc` | string | Yes | Path to the hero image (`/img/hero.png`) |

### ModuleCard (`frontend/src/components/ModuleCard.js`)

**Description**: A reusable card component for the "Master the Stack" section.

| Prop Name | Type | Required | Description |
| :--- | :--- | :--- | :--- |
| `title` | string | Yes | Module title (e.g., "Module 1") |
| `description` | string | Yes | Brief summary of the module content |
| `link` | string | Yes | URL to the module documentation |
| `icon` | string/Element | No | Optional icon or emoji for the card |

## Global Styles (`frontend/src/css/custom.css`)

**CSS Variables**:
- `--ifm-color-primary`: `#F2A900` (Gold)
- `--ifm-background-color`: `#1A1A2E` (Deep Charcoal)
- `--ifm-font-family-base`: `'Source Sans Pro', sans-serif`
- `--ifm-heading-font-family`: `'Playfair Display', serif`
