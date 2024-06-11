export function roundToDigits(num, digits) {
  let x = Math.pow(10, digits);
  if (num instanceof Array) {
    return num.map(n => Math.round(n * x) / x);
  } else {
    return Math.round(num * x) / x;
  }
}


