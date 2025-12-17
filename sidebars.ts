import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/index',
        'module-1/chapter-1',
        'module-1/chapter-2',
        'module-1/chapter-3'
      ],
    },
  ],

  // Manual sidebar for educational content
  docsSidebar: [
    {
      type: 'category',
      label: 'Intro',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/index',
        {
          type: 'category',
          label: 'Chapters',
          items: [
            'module-1/chapter-1',
            'module-1/chapter-2',
            'module-1/chapter-3'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/index',
        {
          type: 'category',
          label: 'Chapters',
          items: [
            'module-2/chapter-1',
            'module-2/chapter-2',
            'module-2/chapter-3'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-isaac-ai-robot/index',
        {
          type: 'category',
          label: 'Chapters',
          items: [
            'module-3-isaac-ai-robot/chapter-1-isaac-sim-fundamentals',
            'module-3-isaac-ai-robot/chapter-2-isaac-ros-perception',
            'module-3-isaac-ai-robot/chapter-3-nav2-navigation'
          ],
        },
      ],
    },
  ],
};

export default sidebars;
