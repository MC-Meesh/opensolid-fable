// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { act } from 'react';
import { createRoot } from 'react-dom/client';
import ErrorBoundary from './ErrorBoundary.jsx';

globalThis.IS_REACT_ACT_ENVIRONMENT = true;

function Bomb({ armed }) {
  if (armed) throw new Error('kaboom');
  return <div data-testid="child">alive</div>;
}

describe('ErrorBoundary', () => {
  let container;
  let root;

  beforeEach(() => {
    vi.spyOn(console, 'error').mockImplementation(() => {});
    container = document.createElement('div');
    document.body.appendChild(container);
    root = createRoot(container);
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
    vi.restoreAllMocks();
  });

  it('renders children when nothing throws', () => {
    act(() => {
      root.render(
        <ErrorBoundary name="Panel">
          <Bomb armed={false} />
        </ErrorBoundary>
      );
    });
    expect(container.textContent).toContain('alive');
    expect(container.querySelector('.error-boundary')).toBeNull();
  });

  it('catches a child crash and shows the named fallback instead of unmounting the app', () => {
    act(() => {
      root.render(
        <ErrorBoundary name="3D viewport">
          <Bomb armed />
        </ErrorBoundary>
      );
    });
    const fallback = container.querySelector('.error-boundary');
    expect(fallback).not.toBeNull();
    expect(fallback.textContent).toContain('3D viewport crashed');
    expect(fallback.textContent).toContain('kaboom');
  });

  it('reset re-mounts the children', () => {
    let armed = true;
    function Toggle() {
      return <Bomb armed={armed} />;
    }
    act(() => {
      root.render(
        <ErrorBoundary name="Panel">
          <Toggle />
        </ErrorBoundary>
      );
    });
    expect(container.querySelector('.error-boundary')).not.toBeNull();

    armed = false;
    const button = container.querySelector('.error-boundary button');
    act(() => {
      button.dispatchEvent(new MouseEvent('click', { bubbles: true }));
    });
    expect(container.querySelector('.error-boundary')).toBeNull();
    expect(container.textContent).toContain('alive');
  });

  it('a crash in one boundary does not affect a sibling boundary', () => {
    act(() => {
      root.render(
        <div>
          <ErrorBoundary name="A">
            <Bomb armed />
          </ErrorBoundary>
          <ErrorBoundary name="B">
            <Bomb armed={false} />
          </ErrorBoundary>
        </div>
      );
    });
    expect(container.textContent).toContain('A crashed');
    expect(container.textContent).toContain('alive');
  });
});
