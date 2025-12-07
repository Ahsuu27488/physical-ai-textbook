// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Custom sidebar for the Physical AI & Humanoid Robotics textbook
  textbookSidebar: [
    'index',
    {
      type: 'category',
      label: 'Module 1 - The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/lesson-1-ros2-architecture',
        'module-1/lesson-2-nodes-topics-services',
        'module-1/lesson-3-actions-and-launch-files',
        'module-1/lesson-4-urdf-and-robot-description',
      ],
      link: {
        type: 'doc',
        id: 'module-1/intro',
      },
    },
    {
      type: 'category',
      label: 'Module 2 - The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/lesson-1-gazebo-basics',
        'module-2/lesson-2-sensor-simulation',
        'module-2/lesson-3-ros-gazebo-integration',
      ],
      link: {
        type: 'doc',
        id: 'module-2/intro',
      },
    },
    {
      type: 'category',
      label: 'Module 3 - The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/lesson-1-isaac-sim-basics',
        'module-3/lesson-2-synthetic-data-generation',
        'module-3/lesson-3-isaac-ros-integration',
      ],
      link: {
        type: 'doc',
        id: 'module-3/intro',
      },
    },
    {
      type: 'category',
      label: 'Module 4 - Vision-Language-Action (VLA)',
      items: [
        'module-4/lesson-1-vision-language-fundamentals',
        'module-4/lesson-2-cognitive-planning',
        'module-4/lesson-3-vla-integration',
      ],
      link: {
        type: 'doc',
        id: 'module-4/intro',
      },
    },
    {
      type: 'category',
      label: 'Weeks 1-2 - Introduction to Physical AI',
      items: [
      ],
      link: {
        type: 'doc',
        id: 'week-01-02/intro',
      },
    },
    {
      type: 'category',
      label: 'Weeks 3-5 - ROS 2 Fundamentals',
      items: [
      ],
      link: {
        type: 'doc',
        id: 'week-03-05/intro',
      },
    },
    {
      type: 'category',
      label: 'Weeks 6-7 - Robot Simulation with Gazebo',
      items: [
      ],
      link: {
        type: 'doc',
        id: 'week-06-07/intro',
      },
    },
    {
      type: 'category',
      label: 'Weeks 8-10 - NVIDIA Isaac Platform',
      items: [
      ],
      link: {
        type: 'doc',
        id: 'week-08-10/intro',
      },
    },
    {
      type: 'category',
      label: 'Weeks 11-12 - Humanoid Robot Development',
      items: [
      ],
      link: {
        type: 'doc',
        id: 'week-11-12/intro',
      },
    },
    {
      type: 'category',
      label: 'Week 13 - Conversational Robotics',
      items: [
      ],
      link: {
        type: 'doc',
        id: 'week-13/intro',
      },
    },
  ],
};

export default sidebars;
