#!/bin/bash
# Deployment script for ROS2 Humanoid Education Docusaurus site to GitHub Pages

# Build the site
npm run build

# Deploy to GitHub Pages
npm run deploy