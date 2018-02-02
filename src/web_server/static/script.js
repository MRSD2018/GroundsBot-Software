/**
 * @fileoverview Runs the grudsby Lights application. The code is executed in the
 * user's browser. It communicates with the App Engine backend, renders output
 * to the screen, and handles user interactions.
 */


grudsby = {};  // Our namespace.


/**
 * Starts the grudsby Lights application. The main entry point for the app.
 * @param {string} eeMapId The Earth Engine map ID.
 * @param {string} eeToken The Earth Engine map token.
 * @param {string} serializedPolygonIds A serialized array of the IDs of the
 *     polygons to show on the map. For example: "['poland', 'moldova']".
 */
grudsby.boot = function(eeMapId, eeToken, serializedPolygonIds) {
  // Load external libraries.
  google.load('visualization', '1.0');
  google.load('jquery', '1');
  google.load('maps', '3', {'other_params': 'libraries=drawing&key=AIzaSyCY0S3eiXE_-2iWY23mN-LRq8YDV57B_us'});

  // Create the grudsby app.
  google.setOnLoadCallback(function() {
    var mapType = grudsby.App.getEeMapType(eeMapId, eeToken);
    var app = new grudsby.App(mapType, JSON.parse(serializedPolygonIds));
  });
};


///////////////////////////////////////////////////////////////////////////////
//                               The application.                            //
///////////////////////////////////////////////////////////////////////////////



/**
 * The main grudsby Lights application.
 * This constructor renders the UI and sets up event handling.
 * @param {google.maps.ImageMapType} mapType The map type to render on the map.
 * @param {Array<string>} polygonIds The IDs of the polygons to show on the map.
 *     For example ['poland', 'moldova'].
 * @constructor
 */
grudsby.App = function(mapType, polygonIds) {
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
  var timer = setInterval(this.checkPlan.bind(this),10000);

  this.saveApproval();

  // Register a click handler to show a panel when the user clicks on a place.
  // this.map.data.addListener('click', this.handlePolygonClick.bind(this));

  // Register a click handler to hide the panel when the user clicks close.
  //$('.panel .close').click(this.hidePanel.bind(this));

  // Register a click handler to expand the panel when the user taps on toggle.
  //$('.panel .toggler').click((function() {
  //  $('.panel').toggleClass('expanded');
  //}).bind(this));
};


/**
 * Creates a Google Map with a black background the given map type rendered.
 * The map is anchored to the DOM element with the CSS class 'map'.
 * @param {google.maps.ImageMapType} mapType The map type to include on the map.
 * @return {google.maps.Map} A map instance with the map type rendered.
 */
grudsby.App.prototype.createMap = function(mapType) {
  var mapOptions = {
    center: grudsby.App.DEFAULT_CENTER,
    disableDefaultUI: true,
    zoom: grudsby.App.DEFAULT_ZOOM
  };
  var mapEl = $('.map').get(0);
  var map = new google.maps.Map(mapEl, mapOptions);
  //map.overlayMapTypes.push(mapType);
  map.setMapTypeId('satellite');
  grudsby.App.polyDrawingManager = new google.maps.drawing.DrawingManager({
      drawingMode: null,//google.maps.drawing.OverlayType.POLYGON,
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

/**
 * Adds the polygons with the passed-in IDs to the map.
 * @param {Array<string>} polygonIds The IDs of the polygons to show on the map.
 *     For example ['poland', 'moldova'].
 */
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
        editable: true
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
      if (grudsby.App.currentPlan!=null)
      {
        grudsby.App.currentPlan.setMap(null);
      };
      grudsby.App.currentPlan = new google.maps.Polyline({
        path: data.coordinates,
        strokeColor: 'red',
        strokeWeight: 2,
        strokeOpacity: 1.0,
        map: this.map,
        editable: false
      });
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
/**
 * Handles a on click a polygon. Highlights the polygon and shows details about
 * it in a panel.
 * @param {Object} event The event object, which contains details about the
 *     polygon clicked.
 */
/*grudsby.App.prototype.handlePolygonClick = function(event) {
  this.clear();
  var feature = event.feature;

  // Instantly higlight the polygon and show the title of the polygon.
  this.map.data.overrideStyle(feature, {strokeWeight: 8});
  var title = feature.getProperty('title');
  $('.panel').show();
  $('.panel .title').show().text(title);

  // Asynchronously load and show details about the polygon.
  var id = feature.getProperty('id');
  $.get('/details?polygon_id=' + id).done((function(data) {
    if (data['error']) {
      $('.panel .error').show().html(data['error']);
    } else {
      $('.panel .wiki-url').show().attr('href', data['wikiUrl']);
      this.showChart(data['timeSeries']);
    }
  }).bind(this));
};*/


/** Clears the details panel and selected polygon. */
/*grudsby.App.prototype.clear = function() {
  $('.panel .title').empty().hide();
  $('.panel .wiki-url').hide().attr('href', '');
  $('.panel .chart').empty().hide();
  $('.panel .error').empty().hide();
  $('.panel').hide();
  this.map.data.revertStyle();
};*/


/** Hides the details panel. */
/*grudsby.App.prototype.hidePanel = function() {
  $('.panel').hide();
  this.clear();
};*/


/**
 * Shows a chart with the given timeseries.
 * @param {Array<Array<number>>} timeseries The timeseries data
 *     to plot in the chart.
 */
/*grudsby.App.prototype.showChart = function(timeseries) {
  timeseries.forEach(function(point) {
    point[0] = new Date(parseInt(point[0], 10));
  });
  var data = new google.visualization.DataTable();
  data.addColumn('date');
  data.addColumn('number');
  data.addRows(timeseries);
  var wrapper = new google.visualization.ChartWrapper({
    chartType: 'LineChart',
    dataTable: data,
    options: {
      title: 'Brightness over time',
      curveType: 'function',
      legend: {position: 'none'},
      titleTextStyle: {fontName: 'Roboto'}
    }
  });
  $('.panel .chart').show();
  var chartEl = $('.chart').get(0);
  wrapper.setContainerId(chartEl);
  wrapper.draw();
};*/




///////////////////////////////////////////////////////////////////////////////
//                        Static helpers and constants.                      //
///////////////////////////////////////////////////////////////////////////////


/**
 * Generates a Google Maps map type (or layer) for the passed-in EE map id. See:
 * https://developers.google.com/maps/documentation/javascript/maptypes#ImageMapTypes
 * @param {string} eeMapId The Earth Engine map ID.
 * @param {string} eeToken The Earth Engine map token.
 * @return {google.maps.ImageMapType} A Google Maps ImageMapType object for the
 *     EE map with the given ID and token.
 */
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

/** @type {Object} The current polygon. */
grudsby.App.currentPlan;

/** @type {Object} Tracks whether the user has approved the region. */
grudsby.App.planApproved = false;

