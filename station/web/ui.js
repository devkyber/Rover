// Presentation-only helpers. They never send rover commands or change inputs.
export function setupOperationDetails(root) {
  const drawers = [...root.querySelectorAll('.hammer-details, .jumper-details')];
  for (const drawer of drawers) {
    const summary = drawer.querySelector(':scope > summary');
    const body = drawer.querySelector('.operation-detail-body');
    const name = drawer.closest('.operation-section').querySelector('h2').textContent.trim();
    const close = document.createElement('button');
    close.type = 'button';
    close.className = 'details-close';
    close.dataset.closeDetails = '';
    close.textContent = '×';
    close.title = 'Close settings (Esc)';
    close.setAttribute('aria-label', `Close ${name} settings`);
    close.addEventListener('click', () => { drawer.open = false; summary.focus({ preventScroll: true }); });
    body.prepend(close);
    drawer.addEventListener('toggle', () => {
      if (drawer.open) for (const other of drawers) if (other !== drawer) other.open = false;
    });
  }
  document.addEventListener('pointerdown', event => {
    for (const drawer of drawers) if (drawer.open && !drawer.contains(event.target)) drawer.open = false;
  });
  window.addEventListener('keydown', event => {
    if (event.code !== 'Escape' || document.fullscreenElement) return;
    const open = drawers.find(drawer => drawer.open && drawer.getClientRects().length);
    if (!open) return;
    event.preventDefault(); event.stopPropagation();
    open.open = false;
    open.querySelector(':scope > summary').focus({ preventScroll: true });
  });
}
