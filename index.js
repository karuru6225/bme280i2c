var i2c = require('i2c');
var async = require('async');

var address = 0x76;
var sensor = new i2c(address, {device: '/dev/i2c-1'});

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

  ch[1] = ushort2Short(ch[1]);
  ch[3] = ushort2Short(ch[3]);
  ch[4] = ushort2Short(ch[4]);
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
        var raw = {};
        sensor.readBytes(0xf7, 8, function(e, val){
          for(var i = 0, len = val.length; i < len; i++){
            raw[0xf7+i] = val[i];
          }
          cb(null, raw);
        })
      },
      function(raw, cb){
        var raws = {
          t: (raw[0xfa] << 12) | (raw[0xfb] << 4) | ((0xf0&raw[0xfc]) >> 4),
          p: (raw[0xf7] << 12) | (raw[0xf8] << 4) | ((0xf0&raw[0xf9]) >> 4),
          h: (raw[0xfd] << 8) | raw[0xfe]
        };
        cb(null, raws);
      }
  ], function(e, raws){
    callback(raws);
  });
}

function init(cb){
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
  //  os_h
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
  //  filter
  //    off : 0
  //    2   : 1
  //    4   : 2
  //    8   : 3
  //    16  : 4
  var stanby = 3,
    os_t = 1,
    os_p = 1,
    os_h = 1,
    mode = 2,
    filter = 0;
  
  // spi3w = 0;(i2cだから)
  var spi3w = 0;

  var config = (stanby&0x7) << 5 | (filter&0x7) << 2 | (spi3w&0x1);
  var ctrl_meas = (os_t&0x7) << 5 | (os_p&0x7) << 2 | (mode&0x3);
  var ctrl_hum = mode&0x7;
  var ostn = (os_t != 0) ? Math.pow(2, os_t - 1) : 0;
  var ospn = (os_p != 0) ? Math.pow(2, os_p - 1) : 0;
  var oshn = (os_h != 0) ? Math.pow(2, os_h - 1) : 0;

  var wait_time = Math.ceil(1.25 + (2.3 * ostn) + (2.3 * ospn + 0.575) + (2.3 * oshn + 0.575));

  async.series([
      function(cb){
        sensor.writeBytes(0xf5, [config], function(){cb()});
      },
      function(cb){
        sensor.writeBytes(0xf4, [ctrl_meas], function(){cb()});
      },
      function(cb){
        sensor.writeBytes(0xf2, [ctrl_hum], function(){cb()});
      }
  ], function(){cb(null, wait_time)});
}

function triggerForce(cb){
  sensor.writeBytes(0xf4, [2], function(){cb()});
}

function measure(){
  async.waterfall([
      function(cb){
        init(cb)
      },
      function(time, cb){
        setTimeout(cb, time);
      },
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
        //console.log((new Date()).toTimeString());
        //console.log(Date.now());
        //console.log('temp:' + (temp/100.0));
        //console.log('pres:' + (pres/100.0));
        //console.log('humi:' + (humi));
        console.log(Date.now()+','+(temp/100.0)+','+(pres/100.0)+','+humi);
        //console.log('');
        cb();
      }
  ]);
}

var calib = null;
getCalibrationData(function(val){
  calib = val;
});

init(function(){
  measure();
  setInterval(measure,60000);
});
