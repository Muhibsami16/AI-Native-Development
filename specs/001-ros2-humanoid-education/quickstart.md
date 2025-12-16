# Quickstart: ROS2 Humanoid Education Module

## Prerequisites

- Node.js 18+ installed
- Git installed
- Basic knowledge of ROS 2 concepts (helpful but not required)

## Installation

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install Docusaurus dependencies**:
   ```bash
   cd website
   npm install
   ```

3. **Start the development server**:
   ```bash
   npm run start
   ```

4. **Open your browser** to `http://localhost:3000` to view the documentation.

## Adding New Content

1. **Create a new module directory** in `docs/`:
   ```bash
   mkdir docs/module-2
   ```

2. **Add chapter files** with proper naming:
   ```bash
   touch docs/module-2/chapter-1.md
   touch docs/module-2/chapter-2.md
   ```

3. **Update the sidebar configuration** in `website/sidebars.js`:
   ```javascript
   module.exports = {
     docs: [
       {
         type: 'category',
         label: 'Module 2',
         items: ['module-2/chapter-1', 'module-2/chapter-2'],
       },
     ],
   };
   ```

## Building for Production

1. **Build the static site**:
   ```bash
   npm run build
   ```

2. **Serve the built site locally for testing**:
   ```bash
   npm run serve
   ```

## Deployment to GitHub Pages

1. **Set up GitHub Pages** in your repository settings to use the `gh-pages` branch

2. **Deploy using the provided script**:
   ```bash
   npm run deploy
   ```

This will build the site and push it to the `gh-pages` branch for GitHub Pages hosting.

## Customizing the Documentation

- Modify `website/docusaurus.config.js` to change site metadata, theme, and plugins
- Update `website/src/css/custom.css` to add custom styling
- Add images and other static assets to `website/static/`