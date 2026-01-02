// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: ['getting-started'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'ros2-nervous-system/concepts',
        'ros2-nervous-system/hands-on-lab',
        'ros2-nervous-system/troubleshooting'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments',
      items: [
        'simulation/concepts',
        'simulation/hands-on-lab',
        'simulation/troubleshooting'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Brain',
      items: [
        'nvidia-isaac-brain/concepts',
        'nvidia-isaac-brain/hands-on-lab',
        'nvidia-isaac-brain/troubleshooting'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Systems',
      items: [
        'vla-systems/concepts',
        'vla-systems/hands-on-lab',
        'vla-systems/troubleshooting'
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone-project/project-overview',
        'capstone-project/implementation-guide',
        'capstone-project/evaluation-criteria'
      ],
    },
    {
      type: 'category',
      label: 'Reference',
      items: [
        'reference/simulation-api-reference',
        'reference/ros2-api-reference',
        'reference/isaac-api-reference',
        'reference/vla-api-reference',
      ],
    },
  ],
};

export default sidebars;