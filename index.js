var i2c = require('i2c');
var async = require('async');

var address = 0x76;
var sensor = new i2c(address, {device: '/dev/i2c-1'});

//  stanby
//    0.5ms : 0
//    62.5ms: 1
//    125ms : 2
//    250ms : 3
//    500ms : 4
//    1000ms: 5
//    10ms  : 6
//    20ms  : 7
//  os_t
//    skip  : 0
//    x1    : 1
//    x2    : 2
//    x4    : 3
//    x8    : 4
//    x16   : 5
//  os_p
//    skip  : 0
//    x1    : 1
//    x2    : 2
//    x4    : 3
//    x8    : 4
//    x16   : 5
//  mode
//    sleep : 0
//    force : 1, 2
//    normal: 3
function initialize(stanby, os_t, os_p, mode, callback){
  //  filter
  //    off : 0
  //    2   : 1
  //    4   : 2
  //    8   : 3
  //    16  : 4
  var filter = 0;
  // spi3w = 0;(i2cだから)
  var spi3w = 0;

  var config = (stanby&0x7) << 5 | (filter&0x7) << 2 | (spi3w&0x1);
  var control = (os_t&0x7) << 5 | (os_p&0x7) << 2 | (mode&0x3);

  async.parallel([
      function(cb){
        sensor.writeBytes(0xf5, [config], function(){cb()});
      },
      function(cb){
        sensor.writeBytes(0xf4, [control], function(){cb()});
      },
    ], function(){ callback() });
}

function getCalibrationData(callback){
  var c = {};
  async.series([
    function(cb){
      sensor.readBytes(0x88, 26, function(e, val){
        for(var i = 0, len = val.length; i < len; i++){
          c[0x88+i] = val[i];
        }
        cb();
      });
    },
    function(cb){
      sensor.readBytes(0xe1, 16, function(e, val){
        for(var i = 0, len = val.length; i < len; i++){
          c[0xe1+i] = val[i];
        }
        cb();
      });
    }
  ], function(){
    callback(c);
  });
}

function getTBase(tr, c){
  var t1 = (c[0x88] | c[0x89] << 8);
  var t2 = ushort2Short(c[0x8a] | c[0x8b] << 8);
  var t3 = ushort2Short(c[0x8c] | c[0x8d] << 8);

  var v1 = (tr>>3) - (t1<<1);
  v1 = (v1 * t2) >> 11;

  var v2 = ( (tr>>4) - t1 ) * ( (tr>>4) - t1 );
  v2 = ( (v2 >> 12) * t3 ) >> 14;

  return v1 + v2;
}

function calibrateT(t_base, c){
  return ( t_base * 5 + 128 ) >> 8;
}

function ushort2Short(num){
  if(num & 0x8000){
    return (-num ^ 0xffff) + 1;
  }
  return num;
}

function uchar2Char(num){
  if(num & 0x80){
    return (-num ^ 0xff) + 1;
  }
  return num;
}

function calibrateP(pr, t_base, c){
  var cp = [];
  cp.push(c[0x8e] | c[0x8f] << 8);
  cp.push(c[0x90] | c[0x91] << 8);
  cp.push(c[0x92] | c[0x93] << 8);
  cp.push(c[0x94] | c[0x95] << 8);
  cp.push(c[0x96] | c[0x97] << 8);
  cp.push(c[0x98] | c[0x99] << 8);
  cp.push(c[0x9a] | c[0x9b] << 8);
  cp.push(c[0x9c] | c[0x9d] << 8);
  cp.push(c[0x9e] | c[0x9f] << 8);

  for(var i = 1; i < cp.length; i++){
    cp[i] = ushort2Short(cp[i]);
  }

  var v1 = (t_base/2.0) - 64000;
  var v2 = v1 * v1 * cp[5] / 32768.0;
  v2 = v2 + v1 * cp[4] * 2;
  v2 = (v2/4.0) + cp[3] * 65536;
  v1 = (cp[2] * v1 * v1 / 524288.0 + cp[1] * v1) / 524288.0;
  v1 = (1 + v1 / 32768.0) * cp[0];
  if( v1 == 0 ){
    return 0;
  }
  var press = 1048576.0 - pr;
  press = (press - (v2/4096)) * 6250 / v1;
  v1 = cp[8] * press * press / 2147483648.0;
  v2 = press * cp[7] / 32768.0;
  return press + (v1 + v2 + cp[6])/16.0;
}

function calibrateH(hr, t_base, c){
  var ch = [];
  ch.push( c[0xa1] );
  ch.push( c[0xe1] | (c[0xe2] << 8) );
  ch.push( c[0xe3] );
  ch.push( (c[0xe4]<<4) | ( (c[0xe5] & 0x0f) ) );
  ch.push( ( (c[0xe5]&0xf0) >> 4 ) | ( c[0xe6] << 4 ) );
  ch.push( c[0xe7] );

  for(var i in [1, 3, 4]){
    ch[i] = ushort2Short(ch[i]);
  }
  ch[5] = uchar2Char(ch[5]);

  var humi;
  humi = ((t_base) - 76800.0);
  humi = (hr - ((ch[3]) * 64.0 + (ch[4]) / 16384.0 * humi)) *
  ((ch[1]) / 65536.0 * (1.0 + (ch[5]) / 67108864.0 * humi *
  (1.0 + (ch[2]) / 67108864.0 * humi)));
  humi = humi * (1.0 - (ch[0]) * humi / 524288.0);
  if (humi > 100.0){
    humi = 100.0;
  }else if (humi < 0.0){
    humi = 0.0;
  }
  return humi;
}

function readRawData(callback){
  async.waterfall([
      function(cb){
        var raw = [];
        sensor.readBytes(0xf7, 8, function(e, val){
          for(var i = 0, len = val.length; i < len; i++){
            raw.push(val[i]);
          }
          cb(null, raw);
        })
      },
      function(raw, cb){
        var raws = {
          t: raw[3] << 12 | raw[4] << 4 | raw[5] >> 4,
          p: raw[0] << 12 | raw[1] << 4 | raw[2] >> 4,
          h: raw[6] << 8 | raw[7]
        };
        cb(null, raws);
      }
  ], function(e, raws){
    callback(raws);
  });
}

var calib = null;
async.series([
    function(cb){
      initialize(5, 1, 1, 3, function(){cb();});
    },
    function(cb){
      getCalibrationData(function(val){
        calib = val;
        cb();
      });
    }
]);

setInterval(function(){
  async.waterfall([
      function(cb){
        readRawData(function(raws){
          cb(null, raws);
        });
      },
      function(raws, cb){
        var tbase = getTBase(raws.t, calib);
        var temp = calibrateT(tbase, calib);
        var pres = calibrateP(raws.p, tbase, calib);
        var humi = calibrateH(raws.h, tbase, calib);
        console.log('temp:' + (temp/100.0));
        console.log('pres:' + (pres/100.0));
        console.log('humi:' + (humi));
        console.log('');
        cb();
      }
  ]);
},1000);
