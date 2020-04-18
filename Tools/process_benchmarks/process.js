var args = [];

process.argv.forEach(function (val, index, array) {
  args.push(val);
});

var fs = require('fs');
var logger = fs.createWriteStream('processed_' + args[2], { flags: 'a' });

var lineReader = require('readline').createInterface({
  input: require('fs').createReadStream(args[2])
});

lineReader.on('line', function (line) {
  var elabLine = line.split('duration: ')[1];
  if (elabLine !== undefined) {
    logger.write(elabLine.split('ms')[0]);
    logger.write('\n');
  }
});
