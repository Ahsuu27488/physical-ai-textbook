# ReactBits Skill

## Overview
ReactBits is a collection of React components and animations for creating visually engaging web experiences, including text animations, interactive elements, and unique UI components. It's perfect for creating modern, engaging UI components with animations.

## Installation
```bash
npm install gsap
```

## Key Components and Usage

### Text Animations

#### SplitText Component
Used for animating text character by character or word by word:
```jsx
import SplitText from "./SplitText";

<SplitText
  text="Hello, you!"
  delay={100}
  duration={0.6}
/>
```

#### Animated Text with GSAP
```jsx
import React from 'react';
import AnimatedText from './AnimatedText';

function App() {
  return (
    <div>
      <AnimatedText
        text="Hello, you!"
        splitType="chars"
        staggerDelay={70}
        duration={2}
        ease="elastic.out(1, 0.3)"
        from={{ opacity: 0, y: 40 }}
        to={{ opacity: 1, y: 0 }}
        threshold={0.1}
        rootMargin="-100px"
        textAlign="center"
      />
    </div>
  );
}

export default App;
```

#### CountUp Component
For animated counting:
```javascript
import CountUp from 'react-countup';

function App() {
  return (
    <div>
      <CountUp end={98} />
    </div>
  );
}
```

### Interactive Components

#### Elastic Slider
```jsx
import ElasticSlider from 'your-package-path';
import { FaMinus } from 'react-icons/fa';
import { FaPlus } from 'react-icons/fa';

function App() {
  return (
    <ElasticSlider
      startingValue={50}
      maxValue={500}
      leftIcon={<FaMinus />}
      rightIcon={<FaPlus />}
    />
  );
}
```

### Navigation Components

#### Pill Nav
A customizable navigation component with styling options.

## GSAP Animation Easing Functions
- `back.out(1.5)`
- `power3.out`
- `power2.out`
- `elastic.out(1,0.5)`
- `bounce.out`

## Integration with Docusaurus
React components from ReactBits can be integrated into Docusaurus by:
1. Creating components in the `src/components/` directory
2. Using the components in MDX files or page components
3. Properly handling client-side rendering with the `ClientOnly` wrapper if needed

## Best Practices
- Use appropriate easing functions for smooth animations
- Control animation timing with duration and delay props
- Implement accessibility considerations for animated content
- Use React's useRef for programmatic control of animations