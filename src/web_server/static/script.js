/**
 * @fileoverview Runs the grudsby application. The code is executed in the
 * user's browser. It communicates with the App Engine backend, renders output
 * to the screen, and handles user interactions.
 */

var RotateIcon = function(options){
    this.options = options || {};
    this.rImg = options.img || new Image();
    this.rImg.src = this.rImg.src || this.options.url;
    this.options.width = this.options.width || this.rImg.width || 52;
    this.options.height = this.options.height || this.rImg.height || 60;
    canvas = document.createElement("canvas");
    canvas.width = this.options.width;
    canvas.height = this.options.height;
    this.context = canvas.getContext("2d");
    this.canvas = canvas;
};
RotateIcon.makeIcon = function(url) {
    
    
    return new RotateIcon({url: url});
};
RotateIcon.prototype.setRotation = function(options){
    var canvas = this.context,
        angle = options.deg ? options.deg * Math.PI / 180:
            options.rad,
        centerX = this.options.width/2,
        centerY = this.options.height/2;

    canvas.clearRect(0, 0, this.options.width, this.options.height);
    canvas.save();
    canvas.translate(centerX, centerY);
    canvas.rotate(angle);
    canvas.translate(-centerX, -centerY);
    canvas.drawImage(this.rImg, 0, 0);
    canvas.restore();
    return this;
};

RotateIcon.prototype.getUrl = function(){
    return this.canvas.toDataURL('image/png');
};

grudsby = {};  // Our namespace.

grudsby.boot = function(eeMapId, eeToken) {
  // Load external libraries.
  google.load('visualization', '1.0');
  google.load('jquery', '1');
  google.load('maps', '3', {'other_params': 'libraries=drawing&key=AIzaSyCY0S3eiXE_-2iWY23mN-LRq8YDV57B_us'});

  // Create the grudsby app.
  google.setOnLoadCallback(function() {
    var mapType = grudsby.App.getEeMapType(eeMapId, eeToken);
    var app = new grudsby.App(mapType);
  });
};


///////////////////////////////////////////////////////////////////////////////
//                               The application.                            //
///////////////////////////////////////////////////////////////////////////////


grudsby.App = function(mapType) {
  // Create and display the map.
  this.map = this.createMap(mapType);

  // Add the polygons to the map.
  this.addPolygon();

  // Register a click handler to toggle polygon drawing mode.
  $('.drawRegion').click(this.drawRegion.bind(this));

  // Register a click handler to toggle polygon drawing mode.
  $('.approvePlan').click(this.approvePlan.bind(this));
  
  // Timer for loading the mowing plan
  this.checkPlan();
  var timer = setInterval(this.checkPlan.bind(this),1000);

  // Write the unapproved state at the start of loading the page.
  this.loadApproval();


  var marker = new google.maps.Marker({
      position: {lat: 40.444505, lng: -79.940777},
      map: this.map,
      title: 'grudsby',
      zIndex:400
    });
  var step = 1;
  var angle = 1;
  setInterval(function(){
    $.get('/mowerPos').done((function(data) {
      if (data['error']) 
      {
        console.log("Failure: %s", data.error);
      }
      else 
      {
        $('.grudsby').attr('src', RotateIcon.makeIcon("/static/grudsby_top_tiny.png")
            .setRotation({deg: -data.rot})
            .getUrl());
        marker.setOptions({
            icon: {
              url:$('.grudsby').attr('src'),
              size: new google.maps.Size(80, 80),
              origin: new google.maps.Point(0, 0),
              anchor: new google.maps.Point(40, 40),
              scaledSize: new google.maps.Size(80, 80) },
            position: {lat: data.lat, lng: data.lng},

        });
      };
    }).bind(this));                           
  }, 500);
       	
            
};


grudsby.App.prototype.createMap = function(mapType) {
  var mapOptions = {
    center: grudsby.App.DEFAULT_CENTER,
    disableDefaultUI: true,
    zoom: grudsby.App.DEFAULT_ZOOM
  };
  var mapEl = $('.map').get(0);
  var map = new google.maps.Map(mapEl, mapOptions);
  //map.overlayMapTypes.push(mapType); // For adding overlays
  map.setMapTypeId('satellite');
  grudsby.App.polyDrawingManager = new google.maps.drawing.DrawingManager({
      drawingMode: null,
      drawingControl: false,
      map: map,
      polygonOptions: {
        fillColor: 'white',
        strokeColor: 'white',
	editable: false
      }
    });
  google.maps.event.addListener(grudsby.App.polyDrawingManager, 'polygoncomplete', this.regionComplete.bind(this));
  return map;
};

grudsby.App.prototype.drawRegion = function() {
  if (grudsby.App.polyDrawingManager.getDrawingMode() == google.maps.drawing.OverlayType.POLYGON) 
  {
    grudsby.App.polyDrawingManager.setOptions({
      drawingMode: null
    });
    $('.drawRegion').get(0).innerHTML="Draw New Region";
    if (grudsby.App.currentRegion!=null){
      grudsby.App.currentRegion.setEditable(true);
    };
  }
  else
  {
    grudsby.App.polyDrawingManager.setOptions({
      drawingMode: google.maps.drawing.OverlayType.POLYGON
    });
    $('.drawRegion').get(0).innerHTML='Edit Current Region';
    if (grudsby.App.currentRegion!=null){
      grudsby.App.currentRegion.setEditable(false);
    };
  };
};

grudsby.App.prototype.regionComplete = function(polygon) {
  grudsby.App.polyDrawingManager.setOptions({
    drawingMode: null
  });
  $('.drawRegion').get(0).innerHTML="Draw New Region";
  if (grudsby.App.currentRegion!=null){
    grudsby.App.currentRegion.setMap(null);
  };
  polygon.setEditable(true);
  polygon.setOptions({
    zIndex:10
  });
  grudsby.App.currentRegion = polygon;
  this.planInvalidated();
  var polyPath = polygon.getPath()
  google.maps.event.addListener(polyPath, 'set_at', this.polyMoved.bind(this));
  google.maps.event.addListener(polyPath, 'insert_at', this.polyMoved.bind(this));
  
};

grudsby.App.prototype.approvePlan = function() {
  if ((grudsby.App.planApproved == false) && (grudsby.App.currentRegion!=null)) {
    $('.approvePlan').get(0).innerHTML="Mowing Plan Approved";
    $('.approvePlan').get(0).setAttribute("style","background-color: #4CAF50");
    grudsby.App.planApproved = true;
    this.saveApproval();
  }
  else {
    this.planInvalidated();
  };
};

grudsby.App.prototype.planInvalidated = function() {
  $('.approvePlan').get(0).innerHTML="Approve Mowing Plan";
  $('.approvePlan').get(0).setAttribute("style","background-color: #f44336");
  grudsby.App.planApproved = false;
  this.saveApproval();
  this.savePolygon();
};

grudsby.App.prototype.addPolygon = function() {
  polygonId = "sve";
  $.get('/region?polygon_id=' + polygonId).done((function(data) {
    if (data['error']) 
    {
      console.log("Failure: %s", data.error);
    }
    else 
    {
      grudsby.App.currentRegion = new google.maps.Polygon({
        paths: data.coordinates,
        fillColor: 'white',
        strokeColor: 'white',
        strokeWeight: 3,
        map: this.map,
        editable: true,
        zIndex: 10
      });
      var polyPath = grudsby.App.currentRegion.getPath()
      google.maps.event.addListener(polyPath, 'set_at', this.polyMoved.bind(this));
      google.maps.event.addListener(polyPath, 'insert_at', this.polyMoved.bind(this));
    };
  }).bind(this)); 
};

grudsby.App.prototype.savePolygon = function() {
  var dataOut = { 
      "coordinates":grudsby.App.currentRegion.getPath().getArray(),
      "regionID":"sve"
    };          

  $.get('/saveRegion?jsonData=' + JSON.stringify(dataOut)).done((function(data) {
      if (data['error']) 
      {
        console.log("Failure: %s", data.error);
      }
      else 
      {
        
      };
    }).bind(this));

};

grudsby.App.prototype.polyMoved = function() {
  this.planInvalidated();
}

grudsby.App.prototype.checkPlan = function() {
  polygonId = "sve";
  $.get('/plan?polygon_id=' + polygonId).done((function(data) {
    if (data['error']) 
    {
      console.log("Failure: %s", data.error);
    }
    else 
    {
      if (grudsby.App.currentPlanText != data)
      {
        var oldPlan = grudsby.App.currentPlan;
        if (data.coordinates.length > 1) 
        { 
          grudsby.App.currentPlan = new google.maps.Polyline({
            path: data.coordinates,
            strokeColor: 'red',
            strokeWeight: 2,
            strokeOpacity: 1.0,
            editable: false,
            zIndex: 100
          });

          grudsby.App.currentPlan.setMap(this.map);
        }
        if (oldPlan!=null)
        {
          oldPlan.setMap(null);
        };
      };
      grudsby.App.currentPlanText = data;
    };
  }).bind(this)); 
}

grudsby.App.prototype.saveApproval = function() {
  var dataOut = { 
      "approval":"false",
      "regionID":"sve"
    };          
  if (grudsby.App.planApproved)
  {
    dataOut.approval = "true";
  };

  $.get('/setApproval?jsonData=' +JSON.stringify(dataOut)).done((function(data) {
    if (data['error']) 
    {
      console.log("Failure: %s", data.error);
    }
    else 
    {
      
    };
  }).bind(this));
}

grudsby.App.prototype.loadApproval = function() {
  $.get('/approval?polygon_id=sve').done((function(data) {
    if (data['error']) 
    {
      console.log("Failure: %s", data.error);
    }
    else 
    {
      grudsby.App.planApproved = false;
      if (data.approval == "true")
      { 
        grudsby.App.planApproved = true;
        $('.approvePlan').get(0).innerHTML="Mowing Plan Approved";
        $('.approvePlan').get(0).setAttribute("style","background-color: #4CAF50");
      }
    };
  }).bind(this)); 
}


///////////////////////////////////////////////////////////////////////////////
//                        Static helpers and constants.                      //
///////////////////////////////////////////////////////////////////////////////


grudsby.App.getEeMapType = function(eeMapId, eeToken) {
  var eeMapOptions = {
    getTileUrl: function(tile, zoom) {
      var url = grudsby.App.EE_URL + '/map/';
      url += [eeMapId, zoom, tile.x, tile.y].join('/');
      url += '?token=' + eeToken;
      return url;
    },
    tileSize: new google.maps.Size(256, 256)
  };
  return new google.maps.ImageMapType(eeMapOptions);
};

/** @type {string} The Earth Engine API URL. */
grudsby.App.EE_URL = 'https://earthengine.googleapis.com';



/** @type {number} The default zoom level for the map. */
grudsby.App.DEFAULT_ZOOM = 20;


/** @type {Object} The default center of the map. */
grudsby.App.DEFAULT_CENTER = {lng: -79.940813, lat: 40.444522};

/** @type {Object} The polygon drawing manager. */
grudsby.App.polyDrawingManager;


/** @type {Object} The current polygon. */
grudsby.App.currentRegion;


grudsby.App.currentPlanText = "";
 

/** @type {Object} The current polygon. */
grudsby.App.currentPlan;

/** @type {Object} Tracks whether the user has approved the region. */
grudsby.App.planApproved = false;

