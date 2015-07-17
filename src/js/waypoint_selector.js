
(function () {
    var canvas = document.getElementById("canvas");
    context = canvas.getContext('2d');

    // resize the canvas to fill browser window dynamically
    window.addEventListener('resize', resizeCanvas, false);

    function resizeCanvas() {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
        drawStuff();
    }

    resizeCanvas();

    function drawStuff() {

    }
})();
