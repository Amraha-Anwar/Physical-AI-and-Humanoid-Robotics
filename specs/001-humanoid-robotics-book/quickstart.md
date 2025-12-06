# Quickstart: Building the Book

## Prerequisites
- Node.js 18+
- npm or yarn

## Installation

1. **Initialize Docusaurus**:
   ```bash
   npx create-docusaurus@latest humanoid-book classic
   cd humanoid-book
   ```

2. **Install Dependencies**:
   ```bash
   npm install
   ```

## Content Integration

1. **Copy Content**:
   Place the generated `.mdx` files into the `docs/` directory following the structure defined in `plan.md`.

2. **Update Sidebar**:
   Replace `sidebars.js` with the contract file provided in `specs/001-humanoid-robotics-book/contracts/sidebar.js`.

## Running Locally

```bash
npm start
```
This opens `http://localhost:3000`.

## Deployment (GitHub Pages)

```bash
GIT_USER=<YourGitHubUsername> USE_SSH=true npm run deploy
```
