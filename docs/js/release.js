document.addEventListener("DOMContentLoaded", function () {
    const version = "v1.0.0";

    const div = document.createElement("div");
    div.innerText = "Release: " + version;

    div.style.position = "fixed";
    div.style.bottom = "50px";   // move above footer
    div.style.left = "20px";
    div.style.fontSize = "12px";
    div.style.opacity = "0.8";
    div.style.zIndex = "9999";
    div.style.background = "rgba(255,255,255,0.9)";
    div.style.padding = "4px 8px";
    div.style.borderRadius = "4px";
    div.style.boxShadow = "0 2px 4px rgba(0,0,0,0.2)";

    document.body.appendChild(div);
});