import React, { useState } from 'react';
import styles from './CodeRunner.module.css';

/**
 * CodeRunner Component
 * Provides an interactive code execution environment for ROS 2 examples
 * Note: This is a frontend component for demonstration. Actual code execution
 * would require backend integration or local execution environment.
 */
const CodeRunner = ({
  initialCode = '# Enter your ROS 2 code here\nimport rclpy\nfrom rclpy.node import Node\n\nclass MinimalPublisher(Node):\n    def __init__(self):\n        super().__init__(\'minimal_publisher\')\n        self.publisher = self.create_publisher(String, \'topic\', 10)\n\nif __name__ == \'__main__\':\n    rclpy.init()\n    node = MinimalPublisher()\n    rclpy.spin(node)',
  language = 'python',
  title = 'ROS 2 Code Runner',
  description = 'Execute ROS 2 Python code directly in the documentation',
  showOutput = true
}) => {
  const [code, setCode] = useState(initialCode);
  const [output, setOutput] = useState('');
  const [isRunning, setIsRunning] = useState(false);
  const [executionTime, setExecutionTime] = useState(0);

  const handleRunCode = async () => {
    setIsRunning(true);
    setOutput('');

    // Simulate code execution (in a real implementation, this would call a backend service)
    const startTime = Date.now();

    // Add a small delay to simulate processing
    await new Promise(resolve => setTimeout(resolve, 1000));

    const endTime = Date.now();
    setExecutionTime(endTime - startTime);

    // Mock output based on code content
    let mockOutput = '';
    if (code.includes('rclpy.init')) {
      mockOutput += 'ROS 2 initialized successfully\n';
    }
    if (code.includes('create_publisher')) {
      mockOutput += 'Publisher created on topic "topic"\n';
    }
    if (code.includes('spin')) {
      mockOutput += 'Node spinning... (this would run indefinitely in real execution)\n';
    }
    if (code.includes('destroy_node')) {
      mockOutput += 'Node destroyed\n';
    }
    if (code.includes('rclpy.shutdown')) {
      mockOutput += 'ROS 2 shutdown\n';
    }

    if (!mockOutput) {
      mockOutput = 'Code executed successfully (no specific ROS 2 operations detected)\n';
    }

    mockOutput += `\nExecution completed in ${executionTime}ms`;

    setOutput(mockOutput);
    setIsRunning(false);
  };

  const handleResetCode = () => {
    setCode(initialCode);
    setOutput('');
  };

  const handleCodeChange = (e) => {
    setCode(e.target.value);
  };

  const copyToClipboard = () => {
    navigator.clipboard.writeText(code);
  };

  return (
    <div className={styles.codeRunnerContainer}>
      <div className={styles.codeRunnerHeader}>
        <div className={styles.codeRunnerTitle}>
          <span className={styles.codeRunnerIcon}>💻</span>
          <h3>{title}</h3>
        </div>
        <p className={styles.codeRunnerDescription}>{description}</p>
      </div>

      <div className={styles.codeRunnerContent}>
        <div className={styles.codeEditor}>
          <div className={styles.editorToolbar}>
            <span className={styles.languageTag}>{language}</span>
            <div className={styles.editorActions}>
              <button
                onClick={copyToClipboard}
                className={styles.toolbarButton}
                title="Copy to clipboard"
              >
                📋 Copy
              </button>
            </div>
          </div>
          <textarea
            value={code}
            onChange={handleCodeChange}
            className={styles.codeTextarea}
            spellCheck={false}
            placeholder="Enter your ROS 2 Python code here..."
          />
        </div>

        {showOutput && (
          <div className={styles.codeOutput}>
            <div className={styles.outputHeader}>
              <span className={styles.outputTitle}>Output</span>
              {executionTime > 0 && (
                <span className={styles.executionTime}>⏱️ {executionTime}ms</span>
              )}
            </div>
            <div className={styles.outputContent}>
              {output || (
                <span className={styles.outputPlaceholder}>
                  Output will appear here after running the code...
                </span>
              )}
            </div>
          </div>
        )}
      </div>

      <div className={styles.codeRunnerControls}>
        <button
          onClick={handleRunCode}
          disabled={isRunning}
          className={`${styles.controlButton} ${styles.runButton}`}
        >
          {isRunning ? '⏳ Running...' : '▶️ Run Code'}
        </button>
        <button
          onClick={handleResetCode}
          className={`${styles.controlButton} ${styles.resetButton}`}
        >
          🔄 Reset
        </button>
        <div className={styles.codeRunnerStatus}>
          <span className={styles.statusIndicator}>
            {isRunning ? '● Running' : '○ Ready'}
          </span>
        </div>
      </div>

      <div className={styles.codeRunnerInfo}>
        <details className={styles.codeRunnerDetails}>
          <summary>Usage Information</summary>
          <div className={styles.codeRunnerDetailsContent}>
            <p><strong>Environment:</strong> Python 3.8+ with ROS 2 Humble</p>
            <p><strong>Libraries:</strong> rclpy, std_msgs, geometry_msgs</p>
            <p><strong>Note:</strong> This is a simulation. Actual ROS 2 code requires a proper ROS 2 environment.</p>
            <p><strong>Safety:</strong> All code execution is simulated for documentation purposes.</p>
          </div>
        </details>
      </div>
    </div>
  );
};

export default CodeRunner;