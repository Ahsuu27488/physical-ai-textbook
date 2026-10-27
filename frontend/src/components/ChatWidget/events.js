// Lets any page open the assistant (and optionally ask a question) without importing the widget.
export const OPEN_EVENT = 'pa:open-assistant';

export function openAssistant(prompt) {
  window.dispatchEvent(new CustomEvent(OPEN_EVENT, {detail: {prompt}}));
}
