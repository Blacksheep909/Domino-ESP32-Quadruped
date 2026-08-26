function niceStep(range, targetIntervals) {
  const roughStep = range / Math.max(1, targetIntervals);
  const exponent = Math.floor(Math.log10(roughStep));
  const magnitude = 10 ** exponent;
  const fraction = roughStep / magnitude;
  const niceFraction = fraction <= 1 ? 1
    : fraction <= 2 ? 2
      : fraction <= 2.5 ? 2.5
        : fraction <= 5 ? 5
          : 10;
  return niceFraction * magnitude;
}

export function niceLinearScale(rawMinimum, rawMaximum, targetIntervals = 4) {
  let minimum = Number(rawMinimum);
  let maximum = Number(rawMaximum);
  if (!Number.isFinite(minimum) || !Number.isFinite(maximum)) return niceLinearScale(-1, 1, targetIntervals);
  if (maximum < minimum) [minimum, maximum] = [maximum, minimum];
  if (Math.abs(maximum - minimum) < 1e-12) {
    const expansion = Math.max(1, Math.abs(minimum) * 0.1);
    minimum -= expansion;
    maximum += expansion;
  }

  const step = niceStep(maximum - minimum, targetIntervals);
  const niceMinimum = Math.floor(minimum / step) * step;
  const niceMaximum = Math.ceil(maximum / step) * step;
  const intervalCount = Math.max(1, Math.round((niceMaximum - niceMinimum) / step));
  const ticks = Array.from({ length: intervalCount + 1 }, (_, index) => {
    const value = niceMinimum + index * step;
    return Math.abs(value) < step * 1e-9 ? 0 : value;
  });
  return { minimum: niceMinimum, maximum: niceMaximum, step, ticks };
}

export function formatAxisValue(value, step) {
  if (!Number.isFinite(value)) return "--";
  const absolute = Math.abs(value);
  if (absolute >= 10_000) {
    const scaled = value / 1_000;
    return `${Number.isInteger(scaled) ? scaled.toFixed(0) : scaled.toFixed(1)}k`;
  }
  const safeStep = Math.abs(Number(step)) || 1;
  const decimals = safeStep >= 1
    ? (Math.abs(safeStep - Math.round(safeStep)) < 1e-9 ? 0 : 1)
    : Math.min(4, Math.max(1, Math.ceil(-Math.log10(safeStep))));
  return value.toFixed(decimals);
}
