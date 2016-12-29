$(document).ready(function() {
  // Start 0.5-second timer to call RESTful endpoint
  setInterval(function() {
    $.ajax({
      url: '/direct/var?key=cortexreply'
      dataType: 'json',
      success: function(json) {
        $('#cortexreply').text(json.result + '% ');
      }
    });
  }, 500);
});
