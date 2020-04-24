function readLines(filename) {
  var timeValues = [];
  var lines = require('fs').readFileSync(filename, 'utf-8')
    .split('\n');

  lines.forEach(line => {
    var elabLine = line.split('duration: ')[1];
    if (elabLine !== undefined) {
      timeValues.push(parseInt(elabLine.split('ms')[0]));
    }
  });

  return timeValues;
}

// Main
(async () => {
  var args = [];
  var timeValues = [];

  process.argv.forEach(val => {
    args.push(val);
  });

  timeValues = readLines(args[2]);
  const avg = (timeValues.reduce((a, b) => a + b, 0) / timeValues.length) || 0;
  const max = Math.max.apply(null, timeValues);
  console.log(`For file ${args[2]}:`);
  console.log(`The average is: ${avg}.`);
  console.log(`The maximum is: ${max}.`);

  process.exit(0);
})();