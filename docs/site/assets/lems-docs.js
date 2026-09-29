/*
 * Small progressive enhancements for the lems-data Doxygen reference.
 *
 * The generated navigation and search implementation remain authoritative.
 * This file only adds a project identity line, an accessible theme preference,
 * and copy controls for generated code fragments.
 */
(function () {
  "use strict";

  var THEME_KEY = "lems-data-docs-theme";
  var THEME_DARK = "dark";
  var THEME_LIGHT = "light";

  function forEachNode(nodes, callback) {
    Array.prototype.forEach.call(nodes, callback);
  }

  function readThemePreference() {
    try {
      var stored = window.localStorage.getItem(THEME_KEY);
      return stored === THEME_DARK || stored === THEME_LIGHT ? stored : null;
    } catch (error) {
      return null;
    }
  }

  function writeThemePreference(theme) {
    try {
      window.localStorage.setItem(THEME_KEY, theme);
    } catch (error) {
      // Private browsing or a restrictive policy may make localStorage
      // unavailable. The current page can still use the selected theme.
    }
  }

  function systemTheme() {
    if (window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches) {
      return THEME_DARK;
    }
    return THEME_LIGHT;
  }

  function updateThemeButton(theme) {
    var button = document.getElementById("lems-theme-toggle");
    if (!button) {
      return;
    }

    var nextLabel = theme === THEME_DARK ? "Switch to light theme" : "Switch to dark theme";
    button.textContent = theme === THEME_DARK ? "Light theme" : "Dark theme";
    button.setAttribute("aria-label", nextLabel);
    button.setAttribute("title", nextLabel);
    button.setAttribute("aria-pressed", theme === THEME_DARK ? "true" : "false");
  }

  function applyTheme(theme, persist) {
    if (theme !== THEME_DARK && theme !== THEME_LIGHT) {
      return;
    }

    document.documentElement.setAttribute("data-lems-theme", theme);
    if (persist) {
      writeThemePreference(theme);
    }
    updateThemeButton(theme);
  }

  function addThemeToggle() {
    if (document.getElementById("lems-theme-toggle")) {
      return;
    }

    var nav = document.getElementById("main-nav-wrapper");
    var mainMenu = document.getElementById("main-menu");
    var listHost = null;
    if (!nav && mainMenu) {
      if (mainMenu.tagName && mainMenu.tagName.toLowerCase() === "ul") {
        listHost = mainMenu;
      } else {
        var childLists = mainMenu.children;
        for (var childIndex = 0; childIndex < childLists.length; childIndex += 1) {
          if (childLists[childIndex].tagName.toLowerCase() === "ul") {
            listHost = childLists[childIndex];
            break;
          }
        }
      }
    }
    nav = nav || listHost || document.getElementById("main-nav");
    if (!nav) {
      return;
    }

    var tools = document.createElement(listHost ? "li" : "div");
    tools.className = "lems-nav-tools";

    var button = document.createElement("button");
    button.type = "button";
    button.id = "lems-theme-toggle";
    button.addEventListener("click", function () {
      var active = document.documentElement.getAttribute("data-lems-theme") || systemTheme();
      applyTheme(active === THEME_DARK ? THEME_LIGHT : THEME_DARK, true);
    });

    tools.appendChild(button);
    nav.appendChild(tools);
    updateThemeButton(document.documentElement.getAttribute("data-lems-theme") || systemTheme());
  }

  function addProjectIdentity() {
    var titleArea = document.getElementById("titlearea");
    if (!titleArea) {
      return;
    }

    var projectName = document.getElementById("projectname");
    if (projectName) {
      if (!projectName.querySelector(".lems-lab-subtitle")) {
        var separator = document.createTextNode(" ");
        var subtitle = document.createElement("span");
        subtitle.className = "lems-lab-subtitle";
        subtitle.textContent = "Brown LEMS · C++ data layer";
        projectName.appendChild(separator);
        projectName.appendChild(subtitle);
      }
      return;
    }

    if (titleArea.querySelector(".lems-doc-brand")) {
      return;
    }

    var brand = document.createElement("div");
    brand.className = "lems-doc-brand";
    brand.innerHTML =
      '<span class="lems-brand-mark" aria-hidden="true">ld</span>' +
      '<span><span class="lems-brand-name">lems-data</span> ' +
      '<span class="lems-lab-subtitle">Brown LEMS · C++ data layer</span></span>';
    titleArea.insertBefore(brand, titleArea.firstChild);
  }

  function isInsideFragment(node) {
    var parent = node.parentElement;
    while (parent) {
      if (parent.classList && parent.classList.contains("fragment")) {
        return true;
      }
      parent = parent.parentElement;
    }
    return false;
  }

  function copySourceText(target) {
    var clone = target.cloneNode(true);
    var controls = clone.querySelectorAll(".lems-copy-code, .lineno");
    forEachNode(controls, function (control) {
      if (control.parentNode) {
        control.parentNode.removeChild(control);
      }
    });

    var lines = clone.querySelectorAll(".line");
    var text;
    if (lines.length) {
      var values = [];
      forEachNode(lines, function (line) {
        values.push(line.textContent);
      });
      text = values.join("\n");
    } else {
      text = clone.textContent;
    }

    return text
      .replace(/\u00a0/g, " ")
      .replace(/\r\n?/g, "\n")
      .replace(/^\n+|\n+$/g, "") + "\n";
  }

  function fallbackCopy(text) {
    var area = document.createElement("textarea");
    var active = document.activeElement;
    area.value = text;
    area.setAttribute("readonly", "");
    area.style.position = "fixed";
    area.style.top = "-1000px";
    area.style.left = "-1000px";
    area.style.opacity = "0";
    document.body.appendChild(area);
    area.focus();
    area.select();

    var selectedText = "";
    try {
      selectedText = area.value.slice(area.selectionStart, area.selectionEnd);
    } catch (error) {
      selectedText = "";
    }

    if (selectedText !== text) {
      document.body.removeChild(area);
      if (active && typeof active.focus === "function") {
        active.focus();
      }
      return false;
    }

    var copied = false;
    try {
      copied = document.execCommand("copy");
    } catch (error) {
      copied = false;
    }

    document.body.removeChild(area);
    if (active && typeof active.focus === "function") {
      active.focus();
    }
    return copied;
  }

  function copyText(text) {
    var clipboard = typeof navigator !== "undefined" ? navigator.clipboard : null;
    if (clipboard && typeof clipboard.writeText === "function") {
      try {
        return Promise.resolve(clipboard.writeText(text)).then(
          function () {
            return true;
          },
          function () {
            return fallbackCopy(text);
          }
        );
      } catch (error) {
        return Promise.resolve(fallbackCopy(text));
      }
    }
    return Promise.resolve(fallbackCopy(text));
  }

  function copyButtonState(button, state) {
    var copied = state === "copied";
    var failed = state === "failed";
    button.textContent = copied ? "Copied" : failed ? "Copy failed" : "Copy code";
    button.setAttribute(
      "aria-label",
      copied ? "Code copied" : failed ? "Copy failed" : "Copy code"
    );
    button.setAttribute("title", copied ? "Code copied" : failed ? "Copy failed" : "Copy code");
    button.classList.toggle("is-copied", copied);
    button.classList.toggle("is-failed", failed);
  }

  function addCopyButton(target) {
    if (target.getAttribute("data-lems-copy-ready") === "true") {
      return;
    }

    var text = copySourceText(target);
    if (!text.trim()) {
      return;
    }

    target.setAttribute("data-lems-copy-ready", "true");
    var button = document.createElement("button");
    button.type = "button";
    button.className = "lems-copy-code";
    button.textContent = "Copy code";
    button.setAttribute("aria-label", "Copy code");
    button.setAttribute("title", "Copy code");
    button.addEventListener("click", function () {
      var copyResult;
      try {
        copyResult = copyText(text);
      } catch (error) {
        copyResult = Promise.resolve(false);
      }
      Promise.resolve(copyResult).then(
        function (copied) {
          copyButtonState(button, copied ? "copied" : "failed");
          window.setTimeout(function () {
            copyButtonState(button, "idle");
          }, copied ? 1800 : 2400);
        },
        function () {
          copyButtonState(button, "failed");
          window.setTimeout(function () {
            copyButtonState(button, "idle");
          }, 2400);
        }
      );
    });
    target.appendChild(button);
  }

  function addCopyButtons() {
    var candidates = document.querySelectorAll(".fragment, pre.prettyprint, pre.code, div.code");
    forEachNode(candidates, function (target) {
      if (
        target.tagName === "PRE" &&
        isInsideFragment(target)
      ) {
        return;
      }
      addCopyButton(target);
    });
  }

  function initialize() {
    var storedTheme = readThemePreference();
    if (storedTheme) {
      applyTheme(storedTheme, false);
    }
    addProjectIdentity();
    addThemeToggle();
    addCopyButtons();
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", initialize);
  } else {
    initialize();
  }
})();
