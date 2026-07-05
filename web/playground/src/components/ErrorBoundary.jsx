import { Component } from 'react';

/**
 * Catches render/lifecycle crashes in one region of the UI so a bug in a
 * single panel (viewport, sketcher, editor) degrades to an inline error
 * card instead of blanking the whole app. "Reset" re-mounts the children.
 */
export default class ErrorBoundary extends Component {
  constructor(props) {
    super(props);
    this.state = { error: null, resetKey: 0 };
    this.reset = this.reset.bind(this);
  }

  static getDerivedStateFromError(error) {
    return { error };
  }

  componentDidCatch(error, info) {
    console.error(`[${this.props.name}] crashed:`, error, info?.componentStack ?? '');
  }

  reset() {
    this.setState((s) => ({ error: null, resetKey: s.resetKey + 1 }));
  }

  render() {
    const { name, children } = this.props;
    const { error, resetKey } = this.state;
    if (error) {
      return (
        <div className="error-boundary" role="alert">
          <strong>{name} crashed</strong>
          <pre>{String(error?.message ?? error)}</pre>
          <button className="secondary" onClick={this.reset}>
            Reset {name}
          </button>
        </div>
      );
    }
    // key forces a full re-mount after reset so the crashed subtree
    // rebuilds its state from scratch instead of re-throwing.
    return <ErrorBoundaryPassthrough key={resetKey}>{children}</ErrorBoundaryPassthrough>;
  }
}

function ErrorBoundaryPassthrough({ children }) {
  return children;
}
