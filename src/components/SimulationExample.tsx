import React from 'react';
import clsx from 'clsx';
import styles from './SimulationExample.module.css';

type SimulationExampleProps = {
  title: string;
  description: string;
  type: 'gazebo' | 'unity' | 'sensor';
  code?: string;
  command?: string;
};

const SimulationTypeColors = {
  gazebo: 'gazebo-section',
  unity: 'unity-section',
  sensor: 'sensor-section',
};

export default function SimulationExample({
  title,
  description,
  type,
  code,
  command,
}: SimulationExampleProps): JSX.Element {
  const typeClass = SimulationTypeColors[type];

  return (
    <div className={clsx('simulation-example', typeClass, styles.simulationExample)}>
      <h3 className={styles.title}>{title}</h3>
      <p className={styles.description}>{description}</p>

      {command && (
        <div className="simulation-terminal">
          <pre>
            <code className="command">$ {command}</code>
          </pre>
        </div>
      )}

      {code && (
        <div className="simulation-code-block">
          <pre>
            <code>{code}</code>
          </pre>
        </div>
      )}
    </div>
  );
}