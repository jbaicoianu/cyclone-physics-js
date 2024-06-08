export function roundToDigits(num, digits) {
  let x = Math.pow(10, digits);
  return Math.round(num * x) / x;
}


