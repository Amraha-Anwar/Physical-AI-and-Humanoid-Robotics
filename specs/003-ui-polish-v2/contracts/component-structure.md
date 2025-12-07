# Component Contract: LearningJourneyMatrix

**Component**: `LearningJourneyMatrix`
**File**: `frontend/src/components/LearningJourneyMatrix.js`

## Props

None (Self-contained data or internal constant).

## DOM Structure

```html
<section class="learning-matrix-section">
  <div class="container">
    <h2 class="matrix-title">Your Learning Journey</h2>
    <div class="matrix-grid">
      <!-- Repeat for each module -->
      <div class="matrix-card">
        <div class="card-header">Module {n}</div>
        <h3 class="card-title">{Title}</h3>
        <p class="card-desc">{Description}</p>
        <a href="{link}" class="card-link">Start Module &rarr;</a>
      </div>
    </div>
  </div>
</section>
```

## CSS Classes (Contract)

- `.learning-matrix-section`: Container section.
- `.matrix-grid`: Grid layout container.
- `.matrix-card`: Individual card style.
- `.matrix-card:hover`: Hover state (gold glow).

# Component Contract: Footer

**Component**: `Footer`
**File**: `frontend/src/theme/Footer/index.js`

## Props

None (Replaces default Docusaurus footer).

## DOM Structure

```html
<footer class="custom-footer">
  <div class="container">
    <div class="footer-row">
      <div class="footer-col-branding">
        <h2 class="footer-title">{Book Title}</h2>
      </div>
      <div class="footer-col-links">
         <!-- Module Links List -->
      </div>
    </div>
    <div class="footer-copyright">
      <!-- Copyright text -->
    </div>
  </div>
</footer>
```
