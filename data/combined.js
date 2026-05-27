// https://cdnjs.cloudflare.com/ajax/libs/reconnecting-websocket/1.0.0/reconnecting-websocket.min.js
!function(a,b){"function"==typeof define&&define.amd?define([],b):"undefined"!=typeof module&&module.exports?module.exports=b():a.ReconnectingWebSocket=b()}(this,function(){function a(b,c,d){function l(a,b){var c=document.createEvent("CustomEvent");return c.initCustomEvent(a,!1,!1,b),c}var e={debug:!1,automaticOpen:!0,reconnectInterval:1e3,maxReconnectInterval:3e4,reconnectDecay:1.5,timeoutInterval:2e3};d||(d={});for(var f in e)this[f]="undefined"!=typeof d[f]?d[f]:e[f];this.url=b,this.reconnectAttempts=0,this.readyState=WebSocket.CONNECTING,this.protocol=null;var h,g=this,i=!1,j=!1,k=document.createElement("div");k.addEventListener("open",function(a){g.onopen(a)}),k.addEventListener("close",function(a){g.onclose(a)}),k.addEventListener("connecting",function(a){g.onconnecting(a)}),k.addEventListener("message",function(a){g.onmessage(a)}),k.addEventListener("error",function(a){g.onerror(a)}),this.addEventListener=k.addEventListener.bind(k),this.removeEventListener=k.removeEventListener.bind(k),this.dispatchEvent=k.dispatchEvent.bind(k),this.open=function(b){h=new WebSocket(g.url,c||[]),b||k.dispatchEvent(l("connecting")),(g.debug||a.debugAll)&&console.debug("ReconnectingWebSocket","attempt-connect",g.url);var d=h,e=setTimeout(function(){(g.debug||a.debugAll)&&console.debug("ReconnectingWebSocket","connection-timeout",g.url),j=!0,d.close(),j=!1},g.timeoutInterval);h.onopen=function(){clearTimeout(e),(g.debug||a.debugAll)&&console.debug("ReconnectingWebSocket","onopen",g.url),g.protocol=h.protocol,g.readyState=WebSocket.OPEN,g.reconnectAttempts=0;var d=l("open");d.isReconnect=b,b=!1,k.dispatchEvent(d)},h.onclose=function(c){if(clearTimeout(e),h=null,i)g.readyState=WebSocket.CLOSED,k.dispatchEvent(l("close"));else{g.readyState=WebSocket.CONNECTING;var d=l("connecting");d.code=c.code,d.reason=c.reason,d.wasClean=c.wasClean,k.dispatchEvent(d),b||j||((g.debug||a.debugAll)&&console.debug("ReconnectingWebSocket","onclose",g.url),k.dispatchEvent(l("close")));var e=g.reconnectInterval*Math.pow(g.reconnectDecay,g.reconnectAttempts);setTimeout(function(){g.reconnectAttempts++,g.open(!0)},e>g.maxReconnectInterval?g.maxReconnectInterval:e)}},h.onmessage=function(b){(g.debug||a.debugAll)&&console.debug("ReconnectingWebSocket","onmessage",g.url,b.data);var c=l("message");c.data=b.data,k.dispatchEvent(c)},h.onerror=function(b){(g.debug||a.debugAll)&&console.debug("ReconnectingWebSocket","onerror",g.url,b),k.dispatchEvent(l("error"))}},1==this.automaticOpen&&this.open(!1),this.send=function(b){if(h)return(g.debug||a.debugAll)&&console.debug("ReconnectingWebSocket","send",g.url,b),h.send(b);throw"INVALID_STATE_ERR : Pausing to reconnect websocket"},this.close=function(a,b){"undefined"==typeof a&&(a=1e3),i=!0,h&&h.close(a,b)},this.refresh=function(){h&&h.close()}}return a.prototype.onopen=function(){},a.prototype.onclose=function(){},a.prototype.onconnecting=function(){},a.prototype.onmessage=function(){},a.prototype.onerror=function(){},a.debugAll=!1,a.CONNECTING=WebSocket.CONNECTING,a.OPEN=WebSocket.OPEN,a.CLOSING=WebSocket.CLOSING,a.CLOSED=WebSocket.CLOSED,a});
// https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.7.3/nipplejs.min.js
!function(t){if("object"==typeof exports&&"undefined"!=typeof module)module.exports=t();else if("function"==typeof define&&define.amd)define([],t);else{var e;e="undefined"!=typeof window?window:"undefined"!=typeof global?global:"undefined"!=typeof self?self:this,e.nipplejs=t()}}(function(){function t(){}function e(t,i){return this.identifier=i.identifier,this.position=i.position,this.frontPosition=i.frontPosition,this.collection=t,this.defaults={size:100,threshold:.1,color:"white",fadeTime:250,dataOnly:!1,restJoystick:!0,restOpacity:.5,mode:"dynamic",zone:document.body,lockX:!1,lockY:!1},this.config(i),"dynamic"===this.options.mode&&(this.options.restOpacity=0),this.id=e.id,e.id+=1,this.buildEl().stylize(),this.instance={el:this.ui.el,on:this.on.bind(this),off:this.off.bind(this),show:this.show.bind(this),hide:this.hide.bind(this),add:this.addToDom.bind(this),remove:this.removeFromDom.bind(this),destroy:this.destroy.bind(this),resetDirection:this.resetDirection.bind(this),computeDirection:this.computeDirection.bind(this),trigger:this.trigger.bind(this),position:this.position,frontPosition:this.frontPosition,ui:this.ui,identifier:this.identifier,id:this.id,options:this.options},this.instance}function i(t,e){var n=this;return n.nipples=[],n.idles=[],n.actives=[],n.ids=[],n.pressureIntervals={},n.manager=t,n.id=i.id,i.id+=1,n.defaults={zone:document.body,multitouch:!1,maxNumberOfNipples:10,mode:"dynamic",position:{top:0,left:0},catchDistance:200,size:100,threshold:.1,color:"white",fadeTime:250,dataOnly:!1,restJoystick:!0,restOpacity:.5,lockX:!1,lockY:!1},n.config(e),"static"!==n.options.mode&&"semi"!==n.options.mode||(n.options.multitouch=!1),n.options.multitouch||(n.options.maxNumberOfNipples=1),n.updateBox(),n.prepareNipples(),n.bindings(),n.begin(),n.nipples}function n(t){var e=this;e.ids={},e.index=0,e.collections=[],e.config(t),e.prepareCollections();var i;return c.bindEvt(window,"resize",function(t){clearTimeout(i),i=setTimeout(function(){var t,i=c.getScroll();e.collections.forEach(function(e){e.forEach(function(e){t=e.el.getBoundingClientRect(),e.position={x:i.x+t.left,y:i.y+t.top}})})},100)}),e.collections}var o,r=!!("ontouchstart"in window),s=!!window.PointerEvent,d=!!window.MSPointerEvent,a={touch:{start:"touchstart",move:"touchmove",end:"touchend, touchcancel"},mouse:{start:"mousedown",move:"mousemove",end:"mouseup"},pointer:{start:"pointerdown",move:"pointermove",end:"pointerup, pointercancel"},MSPointer:{start:"MSPointerDown",move:"MSPointerMove",end:"MSPointerUp"}},p={};s?o=a.pointer:d?o=a.MSPointer:r?(o=a.touch,p=a.mouse):o=a.mouse;var c={};c.distance=function(t,e){var i=e.x-t.x,n=e.y-t.y;return Math.sqrt(i*i+n*n)},c.angle=function(t,e){var i=e.x-t.x,n=e.y-t.y;return c.degrees(Math.atan2(n,i))},c.findCoord=function(t,e,i){var n={x:0,y:0};return i=c.radians(i),n.x=t.x-e*Math.cos(i),n.y=t.y-e*Math.sin(i),n},c.radians=function(t){return t*(Math.PI/180)},c.degrees=function(t){return t*(180/Math.PI)},c.bindEvt=function(t,e,i){for(var n,o=e.split(/[ ,]+/g),r=0;r<o.length;r+=1)n=o[r],t.addEventListener?t.addEventListener(n,i,!1):t.attachEvent&&t.attachEvent(n,i)},c.unbindEvt=function(t,e,i){for(var n,o=e.split(/[ ,]+/g),r=0;r<o.length;r+=1)n=o[r],t.removeEventListener?t.removeEventListener(n,i):t.detachEvent&&t.detachEvent(n,i)},c.trigger=function(t,e,i){var n=new CustomEvent(e,i);t.dispatchEvent(n)},c.prepareEvent=function(t){return t.preventDefault(),t.type.match(/^touch/)?t.changedTouches:t},c.getScroll=function(){return{x:void 0!==window.pageXOffset?window.pageXOffset:(document.documentElement||document.body.parentNode||document.body).scrollLeft,y:void 0!==window.pageYOffset?window.pageYOffset:(document.documentElement||document.body.parentNode||document.body).scrollTop}},c.applyPosition=function(t,e){e.top||e.right||e.bottom||e.left?(t.style.top=e.top,t.style.right=e.right,t.style.bottom=e.bottom,t.style.left=e.left):(t.style.left=e.x+"px",t.style.top=e.y+"px")},c.getTransitionStyle=function(t,e,i){var n=c.configStylePropertyObject(t);for(var o in n)if(n.hasOwnProperty(o))if("string"==typeof e)n[o]=e+" "+i;else{for(var r="",s=0,d=e.length;s<d;s+=1)r+=e[s]+" "+i+", ";n[o]=r.slice(0,-2)}return n},c.getVendorStyle=function(t,e){var i=c.configStylePropertyObject(t);for(var n in i)i.hasOwnProperty(n)&&(i[n]=e);return i},c.configStylePropertyObject=function(t){var e={};return e[t]="",["webkit","Moz","o"].forEach(function(i){e[i+t.charAt(0).toUpperCase()+t.slice(1)]=""}),e},c.extend=function(t,e){for(var i in e)e.hasOwnProperty(i)&&(t[i]=e[i]);return t},c.safeExtend=function(t,e){var i={};for(var n in t)t.hasOwnProperty(n)&&e.hasOwnProperty(n)?i[n]=e[n]:t.hasOwnProperty(n)&&(i[n]=t[n]);return i},c.map=function(t,e){if(t.length)for(var i=0,n=t.length;i<n;i+=1)e(t[i]);else e(t)},t.prototype.on=function(t,e){var i,n=this,o=t.split(/[ ,]+/g);n._handlers_=n._handlers_||{};for(var r=0;r<o.length;r+=1)i=o[r],n._handlers_[i]=n._handlers_[i]||[],n._handlers_[i].push(e);return n},t.prototype.off=function(t,e){var i=this;return i._handlers_=i._handlers_||{},void 0===t?i._handlers_={}:void 0===e?i._handlers_[t]=null:i._handlers_[t]&&i._handlers_[t].indexOf(e)>=0&&i._handlers_[t].splice(i._handlers_[t].indexOf(e),1),i},t.prototype.trigger=function(t,e){var i,n=this,o=t.split(/[ ,]+/g);n._handlers_=n._handlers_||{};for(var r=0;r<o.length;r+=1)i=o[r],n._handlers_[i]&&n._handlers_[i].length&&n._handlers_[i].forEach(function(t){t.call(n,{type:i,target:n},e)})},t.prototype.config=function(t){var e=this;e.options=e.defaults||{},t&&(e.options=c.safeExtend(e.options,t))},t.prototype.bindEvt=function(t,e){var i=this;return i._domHandlers_=i._domHandlers_||{},i._domHandlers_[e]=function(){"function"==typeof i["on"+e]?i["on"+e].apply(i,arguments):console.warn('[WARNING] : Missing "on'+e+'" handler.')},c.bindEvt(t,o[e],i._domHandlers_[e]),p[e]&&c.bindEvt(t,p[e],i._domHandlers_[e]),i},t.prototype.unbindEvt=function(t,e){var i=this;return i._domHandlers_=i._domHandlers_||{},c.unbindEvt(t,o[e],i._domHandlers_[e]),p[e]&&c.unbindEvt(t,p[e],i._domHandlers_[e]),delete i._domHandlers_[e],this},e.prototype=new t,e.constructor=e,e.id=0,e.prototype.buildEl=function(t){return this.ui={},this.options.dataOnly?this:(this.ui.el=document.createElement("div"),this.ui.back=document.createElement("div"),this.ui.front=document.createElement("div"),this.ui.el.className="nipple collection_"+this.collection.id,this.ui.back.className="back",this.ui.front.className="front",this.ui.el.setAttribute("id","nipple_"+this.collection.id+"_"+this.id),this.ui.el.appendChild(this.ui.back),this.ui.el.appendChild(this.ui.front),this)},e.prototype.stylize=function(){if(this.options.dataOnly)return this;var t=this.options.fadeTime+"ms",e=c.getVendorStyle("borderRadius","50%"),i=c.getTransitionStyle("transition","opacity",t),n={};return n.el={position:"absolute",opacity:this.options.restOpacity,display:"block",zIndex:999},n.back={position:"absolute",display:"block",width:this.options.size+"px",height:this.options.size+"px",marginLeft:-this.options.size/2+"px",marginTop:-this.options.size/2+"px",background:this.options.color,opacity:".5"},n.front={width:this.options.size/2+"px",height:this.options.size/2+"px",position:"absolute",display:"block",marginLeft:-this.options.size/4+"px",marginTop:-this.options.size/4+"px",background:this.options.color,opacity:".5"},c.extend(n.el,i),c.extend(n.back,e),c.extend(n.front,e),this.applyStyles(n),this},e.prototype.applyStyles=function(t){for(var e in this.ui)if(this.ui.hasOwnProperty(e))for(var i in t[e])this.ui[e].style[i]=t[e][i];return this},e.prototype.addToDom=function(){return this.options.dataOnly||document.body.contains(this.ui.el)?this:(this.options.zone.appendChild(this.ui.el),this)},e.prototype.removeFromDom=function(){return this.options.dataOnly||!document.body.contains(this.ui.el)?this:(this.options.zone.removeChild(this.ui.el),this)},e.prototype.destroy=function(){clearTimeout(this.removeTimeout),clearTimeout(this.showTimeout),clearTimeout(this.restTimeout),this.trigger("destroyed",this.instance),this.removeFromDom(),this.off()},e.prototype.show=function(t){var e=this;return e.options.dataOnly?e:(clearTimeout(e.removeTimeout),clearTimeout(e.showTimeout),clearTimeout(e.restTimeout),e.addToDom(),e.restCallback(),setTimeout(function(){e.ui.el.style.opacity=1},0),e.showTimeout=setTimeout(function(){e.trigger("shown",e.instance),"function"==typeof t&&t.call(this)},e.options.fadeTime),e)},e.prototype.hide=function(t){var e=this;return e.options.dataOnly?e:(e.ui.el.style.opacity=e.options.restOpacity,clearTimeout(e.removeTimeout),clearTimeout(e.showTimeout),clearTimeout(e.restTimeout),e.removeTimeout=setTimeout(function(){var i="dynamic"===e.options.mode?"none":"block";e.ui.el.style.display=i,"function"==typeof t&&t.call(e),e.trigger("hidden",e.instance)},e.options.fadeTime),e.options.restJoystick&&e.restPosition(),e)},e.prototype.restPosition=function(t){var e=this;e.frontPosition={x:0,y:0};var i=e.options.fadeTime+"ms",n={};n.front=c.getTransitionStyle("transition",["top","left"],i);var o={front:{}};o.front={left:e.frontPosition.x+"px",top:e.frontPosition.y+"px"},e.applyStyles(n),e.applyStyles(o),e.restTimeout=setTimeout(function(){"function"==typeof t&&t.call(e),e.restCallback()},e.options.fadeTime)},e.prototype.restCallback=function(){var t=this,e={};e.front=c.getTransitionStyle("transition","none",""),t.applyStyles(e),t.trigger("rested",t.instance)},e.prototype.resetDirection=function(){this.direction={x:!1,y:!1,angle:!1}},e.prototype.computeDirection=function(t){var e,i,n,o=t.angle.radian,r=Math.PI/4,s=Math.PI/2;if(o>r&&o<3*r&&!t.lockX?e="up":o>-r&&o<=r&&!t.lockY?e="left":o>3*-r&&o<=-r&&!t.lockX?e="down":t.lockY||(e="right"),t.lockY||(i=o>-s&&o<s?"left":"right"),t.lockX||(n=o>0?"up":"down"),t.force>this.options.threshold){var d={};for(var a in this.direction)this.direction.hasOwnProperty(a)&&(d[a]=this.direction[a]);var p={};this.direction={x:i,y:n,angle:e},t.direction=this.direction;for(var a in d)d[a]===this.direction[a]&&(p[a]=!0);if(p.x&&p.y&&p.angle)return t;p.x&&p.y||this.trigger("plain",t),p.x||this.trigger("plain:"+i,t),p.y||this.trigger("plain:"+n,t),p.angle||this.trigger("dir dir:"+e,t)}return t},i.prototype=new t,i.constructor=i,i.id=0,i.prototype.prepareNipples=function(){var t=this,e=t.nipples;e.on=t.on.bind(t),e.off=t.off.bind(t),e.options=t.options,e.destroy=t.destroy.bind(t),e.ids=t.ids,e.id=t.id,e.processOnMove=t.processOnMove.bind(t),e.processOnEnd=t.processOnEnd.bind(t),e.get=function(t){if(void 0===t)return e[0];for(var i=0,n=e.length;i<n;i+=1)if(e[i].identifier===t)return e[i];return!1}},i.prototype.bindings=function(){var t=this;t.bindEvt(t.options.zone,"start"),t.options.zone.style.touchAction="none",t.options.zone.style.msTouchAction="none"},i.prototype.begin=function(){var t=this,e=t.options;if("static"===e.mode){var i=t.createNipple(e.position,t.manager.getIdentifier());i.add(),t.idles.push(i)}},i.prototype.createNipple=function(t,i){var n=this,o=c.getScroll(),r={},s=n.options;if(t.x&&t.y)r={x:t.x-(o.x+n.box.left),y:t.y-(o.y+n.box.top)};else if(t.top||t.right||t.bottom||t.left){var d=document.createElement("DIV");d.style.display="hidden",d.style.top=t.top,d.style.right=t.right,d.style.bottom=t.bottom,d.style.left=t.left,d.style.position="absolute",s.zone.appendChild(d);var a=d.getBoundingClientRect();s.zone.removeChild(d),r=t,t={x:a.left+o.x,y:a.top+o.y}}var p=new e(n,{color:s.color,size:s.size,threshold:s.threshold,fadeTime:s.fadeTime,dataOnly:s.dataOnly,restJoystick:s.restJoystick,restOpacity:s.restOpacity,mode:s.mode,identifier:i,position:t,zone:s.zone,frontPosition:{x:0,y:0}});return s.dataOnly||(c.applyPosition(p.ui.el,r),c.applyPosition(p.ui.front,p.frontPosition)),n.nipples.push(p),n.trigger("added "+p.identifier+":added",p),n.manager.trigger("added "+p.identifier+":added",p),n.bindNipple(p),p},i.prototype.updateBox=function(){var t=this;t.box=t.options.zone.getBoundingClientRect()},i.prototype.bindNipple=function(t){var e,i=this,n=function(t,n){e=t.type+" "+n.id+":"+t.type,i.trigger(e,n)};t.on("destroyed",i.onDestroyed.bind(i)),t.on("shown hidden rested dir plain",n),t.on("dir:up dir:right dir:down dir:left",n),t.on("plain:up plain:right plain:down plain:left",n)},i.prototype.pressureFn=function(t,e,i){var n=this,o=0;clearInterval(n.pressureIntervals[i]),n.pressureIntervals[i]=setInterval(function(){var i=t.force||t.pressure||t.webkitForce||0;i!==o&&(e.trigger("pressure",i),n.trigger("pressure "+e.identifier+":pressure",i),o=i)}.bind(n),100)},i.prototype.onstart=function(t){var e=this,i=e.options;t=c.prepareEvent(t),e.updateBox();var n=function(t){e.actives.length<i.maxNumberOfNipples&&e.processOnStart(t)};return c.map(t,n),e.manager.bindDocument(),!1},i.prototype.processOnStart=function(t){var e,i=this,n=i.options,o=i.manager.getIdentifier(t),r=t.force||t.pressure||t.webkitForce||0,s={x:t.pageX,y:t.pageY},d=i.getOrCreate(o,s);d.identifier!==o&&i.manager.removeIdentifier(d.identifier),d.identifier=o;var a=function(e){e.trigger("start",e),i.trigger("start "+e.id+":start",e),e.show(),r>0&&i.pressureFn(t,e,e.identifier),i.processOnMove(t)};if((e=i.idles.indexOf(d))>=0&&i.idles.splice(e,1),i.actives.push(d),i.ids.push(d.identifier),"semi"!==n.mode)a(d);else{if(!(c.distance(s,d.position)<=n.catchDistance))return d.destroy(),void i.processOnStart(t);a(d)}return d},i.prototype.getOrCreate=function(t,e){var i,n=this,o=n.options;return/(semi|static)/.test(o.mode)?(i=n.idles[0])?(n.idles.splice(0,1),i):"semi"===o.mode?n.createNipple(e,t):(console.warn("Coudln't find the needed nipple."),!1):i=n.createNipple(e,t)},i.prototype.processOnMove=function(t){var e=this,i=e.options,n=e.manager.getIdentifier(t),o=e.nipples.get(n);if(!o)return console.error("Found zombie joystick with ID "+n),void e.manager.removeIdentifier(n);o.identifier=n;var r=o.options.size/2,s={x:t.pageX,y:t.pageY},d=c.distance(s,o.position),a=c.angle(s,o.position),p=c.radians(a),l=d/r;d>r&&(d=r,s=c.findCoord(o.position,d,a));var u=s.x-o.position.x,f=s.y-o.position.y;i.lockX&&(f=0),i.lockY&&(u=0),o.frontPosition={x:u,y:f},i.dataOnly||c.applyPosition(o.ui.front,o.frontPosition);var h={identifier:o.identifier,position:s,force:l,pressure:t.force||t.pressure||t.webkitForce||0,distance:d,angle:{radian:p,degree:a},instance:o,lockX:i.lockX,lockY:i.lockY};h=o.computeDirection(h),h.angle={radian:c.radians(180-a),degree:180-a},o.trigger("move",h),e.trigger("move "+o.id+":move",h)},i.prototype.processOnEnd=function(t){var e=this,i=e.options,n=e.manager.getIdentifier(t),o=e.nipples.get(n),r=e.manager.removeIdentifier(o.identifier);o&&(i.dataOnly||o.hide(function(){"dynamic"===i.mode&&(o.trigger("removed",o),e.trigger("removed "+o.id+":removed",o),e.manager.trigger("removed "+o.id+":removed",o),o.destroy())}),clearInterval(e.pressureIntervals[o.identifier]),o.resetDirection(),o.trigger("end",o),e.trigger("end "+o.id+":end",o),e.ids.indexOf(o.identifier)>=0&&e.ids.splice(e.ids.indexOf(o.identifier),1),e.actives.indexOf(o)>=0&&e.actives.splice(e.actives.indexOf(o),1),/(semi|static)/.test(i.mode)?e.idles.push(o):e.nipples.indexOf(o)>=0&&e.nipples.splice(e.nipples.indexOf(o),1),e.manager.unbindDocument(),/(semi|static)/.test(i.mode)&&(e.manager.ids[r.id]=r.identifier))},i.prototype.onDestroyed=function(t,e){var i=this;i.nipples.indexOf(e)>=0&&i.nipples.splice(i.nipples.indexOf(e),1),i.actives.indexOf(e)>=0&&i.actives.splice(i.actives.indexOf(e),1),i.idles.indexOf(e)>=0&&i.idles.splice(i.idles.indexOf(e),1),i.ids.indexOf(e.identifier)>=0&&i.ids.splice(i.ids.indexOf(e.identifier),1),i.manager.removeIdentifier(e.identifier),i.manager.unbindDocument()},i.prototype.destroy=function(){var t=this;t.unbindEvt(t.options.zone,"start"),t.nipples.forEach(function(t){t.destroy()});for(var e in t.pressureIntervals)t.pressureIntervals.hasOwnProperty(e)&&clearInterval(t.pressureIntervals[e]);t.trigger("destroyed",t.nipples),t.manager.unbindDocument(),t.off()},n.prototype=new t,n.constructor=n,n.prototype.prepareCollections=function(){var t=this;t.collections.create=t.create.bind(t),t.collections.on=t.on.bind(t),t.collections.off=t.off.bind(t),t.collections.destroy=t.destroy.bind(t),t.collections.get=function(e){var i;return t.collections.every(function(t){return!(i=t.get(e))}),i}},n.prototype.create=function(t){return this.createCollection(t)},n.prototype.createCollection=function(t){var e=this,n=new i(e,t);return e.bindCollection(n),e.collections.push(n),n},n.prototype.bindCollection=function(t){var e,i=this,n=function(t,n){e=t.type+" "+n.id+":"+t.type,i.trigger(e,n)};t.on("destroyed",i.onDestroyed.bind(i)),t.on("shown hidden rested dir plain",n),t.on("dir:up dir:right dir:down dir:left",n),t.on("plain:up plain:right plain:down plain:left",n)},n.prototype.bindDocument=function(){var t=this;t.binded||(t.bindEvt(document,"move").bindEvt(document,"end"),t.binded=!0)},n.prototype.unbindDocument=function(t){var e=this;Object.keys(e.ids).length&&!0!==t||(e.unbindEvt(document,"move").unbindEvt(document,"end"),e.binded=!1)},n.prototype.getIdentifier=function(t){var e;return t?void 0===(e=void 0===t.identifier?t.pointerId:t.identifier)&&(e=this.latest||0):e=this.index,void 0===this.ids[e]&&(this.ids[e]=this.index,this.index+=1),this.latest=e,this.ids[e]},n.prototype.removeIdentifier=function(t){var e={};for(var i in this.ids)if(this.ids[i]===t){e.id=i,e.identifier=this.ids[i],delete this.ids[i];break}return e},n.prototype.onmove=function(t){return this.onAny("move",t),!1},n.prototype.onend=function(t){return this.onAny("end",t),!1},n.prototype.oncancel=function(t){return this.onAny("end",t),!1},n.prototype.onAny=function(t,e){var i,n=this,o="processOn"+t.charAt(0).toUpperCase()+t.slice(1);e=c.prepareEvent(e);var r=function(t,e,i){i.ids.indexOf(e)>=0&&(i[o](t),t._found_=!0)},s=function(t){i=n.getIdentifier(t),c.map(n.collections,r.bind(null,t,i)),t._found_||n.removeIdentifier(i)};return c.map(e,s),!1},n.prototype.destroy=function(){var t=this;t.unbindDocument(!0),t.ids={},t.index=0,t.collections.forEach(function(t){t.destroy()}),t.off()},n.prototype.onDestroyed=function(t,e){var i=this;if(i.collections.indexOf(e)<0)return!1;i.collections.splice(i.collections.indexOf(e),1)};var l=new n;return{create:function(t){return l.create(t)},factory:l}});

var ge1doot = ge1doot || {}
ge1doot.canvas = function (elem) {
  var canvas = { width: 0, height: 0, left: 0, top: 0, ctx: null, elem: null }
  canvas.elem = elem
  canvas.elem.onselectstart = function () {
    return false
  }
  canvas.elem.ondragstart = function () {
    return false
  }
  canvas.ctx = canvas.elem.getContext('2d')
  canvas.dpr = window.devicePixelRatio || 1;
  canvas.setSize = function () {
    var o = this.elem
    var w = this.elem.offsetWidth
    var h = this.elem.offsetHeight
    if (w != this.width || h != this.height) {
      for (this.left = 0, this.top = 0; o != null; o = o.offsetParent) {
        this.left += o.offsetLeft
        this.top += o.offsetTop
      }
      this.elem.width = w*this.dpr
      this.elem.height = h*this.dpr
      this.width = w
      this.height = h
      canvas.ctx.scale(this.dpr, this.dpr)
      this.resize && this.resize()
    }
  }
  canvas.setSize()
  canvas.pointer = {
    x: 0,
    y: 0,
    dx: 0,
    dy: 0,
    startX: 0,
    startY: 0,
    canvas: canvas,
    touchMode: false,
    isDown: false,
    center: function (s) {
      this.dx *= s
      this.dy *= s
      endX = endY = 0
    },
    sweeping: false
  }
  var started = false,
    endX = 0,
    endY = 0

  if (window['IN_RB_GRID_DESIGNER'] === true) {
    return canvas
  }

  var addEvent = function (elem, e, fn) {
    for (var i = 0, events = e.split(','); i < events.length; i++) {
      elem.addEventListener(events[i], fn.bind(canvas.pointer), false)
    }
  }
  addEvent(window, 'mousemove,touchmove', function (e) {
    this.touchMode = e.targetTouches
    var pointer = this.touchMode ? this.touchMode[0] : e
    this.x = pointer.clientX - this.canvas.left
    this.y = pointer.clientY - this.canvas.top
    if (started) {
      e.preventDefault()
      this.sweeping = true
      this.dx = endX - (this.x - this.startX)
      this.dy = endY - (this.y - this.startY)
    }
    if (this.move) this.move(e)
  })
  addEvent(canvas.elem, 'mousedown,touchstart', function (e) {
    e.preventDefault()
    this.touchMode = e.targetTouches
    var pointer = this.touchMode ? this.touchMode[0] : e
    this.startX = this.x = pointer.clientX - this.canvas.left
    this.startY = this.y = pointer.clientY - this.canvas.top
    started = true
    this.isDown = true
    if (this.down) this.down(e)
    setTimeout(
      function () {
        if (
          !started &&
          Math.abs(this.startX - this.x) < 11 &&
          Math.abs(this.startY - this.y) < 11
        ) {
          if (this.tap) this.tap(e)
        }
      }.bind(this),
      200
    )
  })
  addEvent(window, 'mouseup,touchend,touchcancel', function (e) {
    if (started) {
      e.preventDefault()
      endX = this.dx
      endY = this.dy
      started = false
      this.isDown = false
      if (this.up) this.up(e)
      this.sweeping = false
    }
  })
  return canvas
}

if (String.prototype.endsWith === undefined) {
  String.prototype.endsWith = function (suffix) {
    return this.indexOf(suffix, this.length - suffix.length) !== -1
  }
}

if (typeof Object.assign !== 'function') {
  // Must be writable: true, enumerable: false, configurable: true
  Object.defineProperty(Object, 'assign', {
    value: function assign(target, varArgs) {
      // .length of function is 2
      'use strict'
      if (target === null || target === undefined) {
        throw new TypeError('Cannot convert undefined or null to object')
      }

      var to = Object(target)

      for (var index = 1; index < arguments.length; index++) {
        var nextSource = arguments[index]

        if (nextSource !== null && nextSource !== undefined) {
          for (var nextKey in nextSource) {
            // Avoid bugs when hasOwnProperty is shadowed
            if (Object.prototype.hasOwnProperty.call(nextSource, nextKey)) {
              to[nextKey] = nextSource[nextKey]
            }
          }
        }
      }
      return to
    },
    writable: true,
    configurable: true
  })
}

function Log(elementId) {
  this.el = document.getElementById(elementId)
  this.open = false
  this.isTouched = false
  this.touchStart = 0
  this.lastClick = 0

  this.el.addEventListener('click', this.onClick.bind(this))

  this.el.addEventListener(
    'touchstart',
    function (ev) {
      this.isTouched = true
      this.touchStart = ev.timeStamp
    }.bind(this)
  )
  this.el.addEventListener(
    'touchend',
    function (ev) {
      this.isTouched = false
      if (ev.timeStamp - this.touchStart < 400) {
        this.onClick(ev)
      }
    }.bind(this)
  )

  this.scrollToBottom()
}

Log.prototype.onClick = function (ev) {
  if (ev.timeStamp - this.lastClick < 500) return
  this.lastClick = ev.timeStamp

  this.open = !this.open
  if (this.open) {
    this.el.classList.replace('log-short', 'log-full')
  } else {
    this.el.classList.replace('log-full', 'log-short')
  }
  this.scrollToBottom()
}

Log.prototype.scrollToBottom = function () {
  this.el.scrollTop = this.el.scrollHeight
}

Log.prototype.clear = function () {
  this.el.textContent = ''
}

Log.prototype.write = function (msg, noNewLine) {
  if (noNewLine !== true && !msg.endsWith('\n')) {
    msg += '\n'
  }
  var content = this.el.textContent
  if (content.length > 8192) {
    content = content.substring(4096)
  }
  this.el.textContent = content + msg
  if (!this.isTouched) {
    this.scrollToBottom()
  }
}

function Manager(logElementId, gridElementId) {
  this.socket = null

  this.divider = 0

  this.mustArriveIdIn = -1
  this.mustArriveIdOut = 0
  this.mustArriveCommands = {}
  this.MUST_ARRIVE_TIMER_FULL = 100
  this.MUST_ARRIVE_RETRIES = 30

  this.mustArriveTimerId = null
  this.possessed = false
  this.isOverWebSocket = false
  this.pktCounter = 0

  this.log = new Log(logElementId)
  this.grid = null

  this.loadLayout(gridElementId)
}

Manager.prototype.start = function (address, isOverWebSocket) {
  if(this.socket !== null) {
    return
  }

  this.isOverWebSocket = isOverWebSocket === true

  this.log.write('Connecting to ' + address + '... ', true)

  if (!('WebSocket' in window)) {
    this.log.write('\nWebSockets are not supported on this device!')
    return
  }

  this.socket = new ReconnectingWebSocket(address)
  this.socket.addEventListener(
    'open',
    function (event) {
      if (this.mustArriveTimerId === null) {
        this.mustArriveTimerId = setInterval(
          this.mustArriveTask.bind(this),
          this.MUST_ARRIVE_TIMER_FULL
        )
      }

      this.log.write('connected!')
      this.log.write('Attempting to possess the robot...')
      this.sendMustArrive(
        'possess',
        {},
        true,
        function () {
          this.possessed = true
          if (this.grid) {
            this.sendMustArrive('_gall', {})
          }
        }.bind(this)
      )
    }.bind(this)
  )

  this.socket.addEventListener(
    'error',
    function (event) {
      this.log.write('Connection FAILED!')
      if (this.mustArriveTimerId !== null) {
        clearInterval(this.mustArriveTimerId)
        this.mustArriveTimerId = null
      }
    }.bind(this)
  )

  this.socket.addEventListener('message', this.onMessage.bind(this))

  this.lastUpdate = Date.now()
  requestAnimationFrame(this.update.bind(this))
}

Manager.prototype.update = function () {
  var targetDivider = this.isOverWebSocket !== true ? 2 : 4;
  if (++this.divider >= targetDivider) {
    this.divider = 0
  } else {
    requestAnimationFrame(this.update.bind(this))
    return
  }

  var now = Date.now()
  var diff = now - this.lastUpdate
  this.lastUpdate = now

  if (this.socket.readyState === WebSocket.OPEN && this.grid !== null) {
    this.grid.update(diff)
  }

  requestAnimationFrame(this.update.bind(this))
}

Manager.prototype.mustArriveTask = function () {
  for (var id in this.mustArriveCommands) {
    if (!this.mustArriveCommands.hasOwnProperty(id)) continue

    var info = this.mustArriveCommands[id]
    if(this.isOverWebSocket !== true) {
      this.socket.send(info.payload)
    }
    if (info.attempts !== null && ++info.attempts >= this.MUST_ARRIVE_RETRIES) {
      delete this.mustArriveCommands[id]
    }
  }
}

Manager.prototype.onMessage = function (event) {
  try {
    var data = JSON.parse(event.data)
  } catch(e) {
    console.error("Failed to parse message", e)
    return;
  }

  if ('f' in data) {
    var cmd = this.mustArriveCommands[data['f']]
    if (cmd !== undefined) {
      delete this.mustArriveCommands[data['f']]
      if (cmd['callback'] !== undefined) {
        cmd['callback']()
      }
    }
    return
  } else if ('e' in data) {
    var ack = { c: data['c'], e: data['e']  }
    if(this.isOverWebSocket === true) {
      ack['n'] = this.pktCounter++;
    }
    this.socket.send(JSON.stringify(ack))
    var e = data['e']
    if (e <= this.mustArriveIdIn && e !== 0) {
      return
    } else {
      this.mustArriveIdIn = e
    }
  }

  switch (data['c']) {
    case 'pong':
      break
    case 'log':
      this.log.write(data['msg'])
      break
    case '_gst':
      if (this.grid) this.grid.onMessageState(data)
      break
    case '_gtb':
      if (this.grid) this.grid.setCurrentTab(data['tab'])
      break
  }
}

Manager.prototype.send = function (command, data) {
  data['c'] = command
  if(this.isOverWebSocket === true) {
    data['n'] = this.pktCounter++;
  }
  this.socket.send(JSON.stringify(data))
}

Manager.prototype.sendMustArrive = function (
  command,
  data,
  unlimitedAttempts,
  callback
) {
  var id = ++this.mustArriveIdOut
  data['c'] = command
  data['f'] = id
  if(this.isOverWebSocket === true) {
    data['n'] = this.pktCounter++;
  }

  var payload = JSON.stringify(data)
  this.mustArriveCommands[id] = {
    payload: payload,
    attempts: unlimitedAttempts !== true ? 0 : null,
    callback: callback
  }
  this.socket.send(payload)
}

Manager.prototype.flashBody = function () {
  var body = document.getElementById('body')
  body.style.backgroundColor = '#ff5454'
  setTimeout(function () {
    body.style.backgroundColor = 'white'
  }, 50)
}

Manager.prototype.loadLayout = function (gridElementId) {
  //this.grid = new Grid(this, gridElementId, GRID_DATA);
  //return;

  var req = new XMLHttpRequest()
  req.open('GET', '/layout.json')
  req.timeout = 15000
  req.onreadystatechange = function () {
    if (req.readyState !== XMLHttpRequest.DONE) return

    if (req.status !== 200) {
      setTimeout(this.loadLayout.bind(this, gridElementId), 500)
      return
    }

    this.grid = new Grid(this, gridElementId, JSON.parse(req.responseText))
    if (this.possessed) {
      this.sendMustArrive('_gall', {})
    }
  }.bind(this)
  req.send()
}

function Prop(type, getFunc, setFunc) {
  this.type = type
  this.get = getFunc
  this.set = setFunc
  this.editable = true
  this.ignoreInBuilder = false
  this.options = null
  this.isColor = false
  this.step = undefined
}

Prop.prototype.disableEdit = function () {
  this.editable = false
  return this
}

Prop.prototype.setIgnoreInBuilder = function () {
  this.ignoreInBuilder = true
  return this
}

Prop.prototype.setOptions = function (opts) {
  this.options = opts
  return this
}

Prop.prototype.setIsColor = function () {
  this.isColor = true
  return this
}

Prop.prototype.setStep = function (step) {
  this.step = step
  return this
}

function Position(x, y, w, h) {
  this.x = x
  this.y = y
  this.w = w
  this.h = h
}

Position.prototype.equals = function (o) {
  return this.x === o.x && this.y === o.y && this.w === o.w && this.h === o.h
}

function Widget(grid, uuid, element) {
  this.x = 0
  this.y = 0
  this.w = 2
  this.h = 2
  this.tab = 0

  this.grid = grid
  this.uuid = uuid
  this.el = element
  this.eventListener = null
  this.extraCss = {}

  this.el.style.position = 'absolute'
  this.el.classList.add('grid-widget')
}

Widget.SUBCLASSES = []

Widget.prototype.MIN_LIBRARY_VERSION = 0x040000

Widget.prototype.PROPERTIES = {
  id: new Prop(String).setIgnoreInBuilder(),
  x: new Prop(Number).setIgnoreInBuilder(),
  y: new Prop(Number).setIgnoreInBuilder(),
  w: new Prop(Number).setIgnoreInBuilder(),
  h: new Prop(Number).setIgnoreInBuilder(),
  tab: new Prop(Number).setIgnoreInBuilder(),
  css: new Prop(
    Object,
    function () {
      return this.extraCss
    },
    function (val) {
      for (var k in this.extraCss) {
        if (!this.extraCss.hasOwnProperty(k)) continue
        this.el.style.removeProperty(k)
      }
      this.extraCss = {}
      for (var k in val) {
        if (!val.hasOwnProperty(k)) continue
        this.el.style.setProperty(k, val[k], 'important')
        this.extraCss[k] = val[k]
      }
    }
  )
}

Widget.createSubclass = function (cls, properties) {
  cls.prototype = Object.create(Widget.prototype)
  Object.defineProperty(cls.prototype, 'constructor', {
    value: cls,
    enumerable: false,
    writable: true
  })

  Widget.SUBCLASSES.push(cls)

  cls.prototype.PROPERTIES = {}
  Object.assign(cls.prototype.PROPERTIES, Widget.prototype.PROPERTIES)
  if (properties) {
    Object.assign(cls.prototype.PROPERTIES, properties)
  }
}

Widget.wrapCanvas = function (canvas) {
  var wrapper = document.createElement('div')
  canvas.style.width = '100%'
  canvas.style.height = '100%'
  wrapper.appendChild(canvas)
  return wrapper
}

Widget.prototype.applyState = function (state) {
  var proto = Object.getPrototypeOf(this)
  var pos = false
  for (var k in state) {
    if (!state.hasOwnProperty(k) || !proto.PROPERTIES.hasOwnProperty(k)) {
      continue
    }

    var prop = proto.PROPERTIES[k]
    if (prop.set === undefined) {
      this[k] = prop.type(state[k])
    } else {
      prop.set.call(this, state[k])
    }

    if (k.length === 1 && 'xywh'.indexOf(k) !== -1) {
      pos = true
    }
  }

  if (pos) {
    this.updatePosition()
  }
}

Widget.prototype.setEventListener = function (listener) {
  this.eventListener = listener
}

Widget.prototype.updatePosition = function () {
  var pos = this.grid.calculatePxPos(this)
  this.el.style.left = '' + pos.x + 'px'
  this.el.style.top = '' + pos.y + 'px'
  this.el.style.width = '' + pos.w + 'px'
  this.el.style.height = '' + pos.h + 'px'
}

Widget.prototype.update = function (diffMs) {}

Widget.prototype.sendEvent = function (name, extra, mustArrive, callback) {
  if (this.eventListener === null) return
  this.eventListener(
    this,
    name,
    extra ? extra : {},
    mustArrive !== false,
    callback
  )
}

Widget.prototype.pos = function () {
  return new Position(this.x, this.y, this.w, this.h)
}

Widget.prototype.getState = function () {
  var res = {}
  var proto = Object.getPrototypeOf(this)
  for (var key in proto.PROPERTIES) {
    if (!proto.PROPERTIES.hasOwnProperty(key)) continue

    var prop = proto.PROPERTIES[key]
    if (prop.get === undefined) {
      res[key] = prop.type(this[key])
    } else {
      res[key] = prop.get.call(this)
    }
  }
  return res
}

function clampAng(val) {
  val = val % (Math.PI * 2)
  if (val < -Math.PI) val += Math.PI * 2
  else if (val > Math.PI) val -= Math.PI * 2
  return val
}

function deg(rad) {
  return rad * (180.0 / Math.PI)
}

function Bone(info, color, prev) {
  this.length = info.len
  this.color = color

  this.x = 0
  this.y = 0
  this.angle = info.angle
  if (prev === null) {
    this.relAngle = this.angle
  } else {
    this.relAngle = clampAng(this.angle - prev.angle)
  }

  this.relMin = info.rmin
  this.relMax = info.rmax
  this.absMin = info.amin
  this.absMax = info.amax
  this.baseMin = info.bmin
  this.baseMax = info.bmax
}

Bone.prototype.toInfo = function () {
  return {
    len: this.length,
    angle: this.angle,
    rmin: this.relMin,
    rmax: this.relMax,
    amin: this.absMin,
    amax: this.absMax,
    bmin: this.baseMin,
    bmax: this.baseMax
  }
}

Bone.prototype.updatePos = function (prevBone, unit) {
  this.angle = this.relAngle
  if (prevBone) {
    this.angle = clampAng(prevBone.angle + this.angle)
  }

  this.x = Math.cos(this.angle) * this.length * unit
  this.y = Math.sin(this.angle) * this.length * unit

  if (prevBone) {
    this.x += prevBone.x
    this.y += prevBone.y
  }
}

function Animation(arm) {
  this.arm = arm
  this.keyframes = []
  this.curFrame = -1
  this.lastTick = null
  this.cmdSent = true
}

Animation.prototype.addFrame = function (x, y, durationMs) {
  this.keyframes.push({
    x: x,
    y: y,
    duration: durationMs,
    current: 0
  })
}

Animation.prototype.start = function () {
  this.lastTick = performance.now()
  this.nextFrame()
  requestAnimationFrame(this.update.bind(this))
}

Animation.prototype.nextFrame = function () {
  this.curFrame += 1
  if (this.curFrame >= this.keyframes.length) return false
  var f = this.keyframes[this.curFrame]

  var b = this.arm.bones[this.arm.bones.length - 1]
  f.start_x = b.x / this.arm.unit
  f.start_y = b.y / this.arm.unit

  this.arm.pointer.x = f.x * this.arm.unit + this.arm.origin.x
  this.arm.pointer.y = f.y * this.arm.unit + this.arm.origin.y
  this.arm.run()

  this.cmdSent = false

  this.arm.sendEvent(
    'pos',
    { armX: f.x, armY: f.y },
    true,
    function () {
      this.cmdSent = true
      requestAnimationFrame(this.update.bind(this))
    }.bind(this)
  )

  return true
}

Animation.prototype.update = function () {
  var now = performance.now()
  var diff = now - this.lastTick
  this.lastTick = now

  if (!this.cmdSent) return

  if (this.curFrame >= this.keyframes.length) {
    this.arm.animation = null
    return
  }

  var f = this.keyframes[this.curFrame]
  f.current += diff

  var progress = f.current / f.duration
  if (progress >= 1.0) progress = 1.0

  if (progress >= 1.0 && !this.nextFrame()) {
    this.arm.animation = null
    return
  }

  requestAnimationFrame(this.update.bind(this))
}

function Arm(grid, uuid) {
  var el = document.createElement('canvas')
  Widget.call(this, grid, uuid, Widget.wrapCanvas(el))

  this.w = 12
  this.h = 9

  this.BODY_HEIGHT = 0
  this.BODY_RADIUS = 0
  this.ARM_BASE_HEIGHT = 0
  this.TOUCH_TARGET_SIZE = 4
  this.ARM_TOTAL_LEN = 0

  this.bones = []

  this.BUTTON_TEXTS = ['RETRACT', 'EXTEND', 'GRAB']
  this.buttons = []
  for (var i = 0; i < this.BUTTON_TEXTS.length; ++i) {
    this.buttons.push({
      text: this.BUTTON_TEXTS[i],
      x: 0,
      y: 0,
      w: 0,
      h: 0,
      blink: false
    })
  }

  this.canvas = ge1doot.canvas(el)
  this.canvas.resize = this.resize.bind(this)

  this.unit = 1
  this.origin = { x: 0, y: 0 }
  this.pointer = this.canvas.pointer

  this.touched = false
  this.touchedButton = null
  this.animation = null

  this.pointer.down = function () {
    this.touchedButton = this.getTouchedButton()
    if (this.touchedButton === null && this.animation === null) {
      this.run()
      this.touched = true
    }
  }.bind(this)
  this.pointer.up = function () {
    if (this.touchedButton !== null) {
      if (this.getTouchedButton() === this.touchedButton) {
        this.touchedButton.blink = true
        this.draw()
        setTimeout(
          function (btn) {
            btn.blink = false
            this.draw()
          }.bind(this, this.touchedButton),
          100
        )
        this.handleButton(this.touchedButton.text.toUpperCase())
      }
      this.touchedButton = null
    } else {
      this.touched = false
      requestAnimationFrame(this.run.bind(this))
    }
  }.bind(this)
  this.pointer.move = function () {
    if (this.touched) requestAnimationFrame(this.run.bind(this))
  }.bind(this)

  this.resize()
  this.updateAngles(true)

  this.touched = false
}

Widget.createSubclass(Arm, {
  info: new Prop(
    Object,
    function () {
      var bones = []
      for (var i = 0; i < this.bones.length; ++i)
        bones.push(this.bones[i].toInfo())
      return {
        radius: this.BODY_RADIUS,
        height: this.BODY_HEIGHT,
        off_y: this.ARM_BASE_HEIGHT,
        bones: bones
      }
    },
    function (info) {
      this.BODY_RADIUS = info.radius
      this.BODY_HEIGHT = info.height
      this.ARM_BASE_HEIGHT = info.off_y
      this.ARM_TOTAL_LEN = 0

      this.bones = []
      var colors = ['blue', 'orange', 'green', 'red', 'brown']
      var prev = null
      for (var i = 0; i < info.bones.length; ++i) {
        this.ARM_TOTAL_LEN += info.bones[i].len
        prev = new Bone(info.bones[i], colors[i % colors.length], prev)
        this.bones.push(prev)
      }
    }
  ).disableEdit()
})

Arm.prototype.applyState = function (state) {
  Widget.prototype.applyState.call(this, state)

  this.resize()
  this.updateAngles(true)
}

Arm.prototype.shouldSend = function () {
  return this.bones !== null && this.animation === null
}

Arm.prototype.resize = function () {
  this.unit =
    Math.min(this.canvas.width * 0.6, this.canvas.height * 0.8) /
    this.ARM_TOTAL_LEN

  var w = this.canvas.width / this.buttons.length
  var h = this.canvas.height * 0.15
  var y = this.canvas.height - h
  var x = 0
  for (var i = 0; i < this.buttons.length; ++i) {
    var b = this.buttons[i]
    b.x = x
    b.y = y
    b.w = w
    b.h = h
    x += w
  }

  this.origin.x = this.BODY_RADIUS * this.unit
  this.origin.y =
    y - this.BODY_HEIGHT * 1.3 * this.unit - this.unit * this.ARM_BASE_HEIGHT

  this.draw()
}

Arm.prototype.updatePosition = function (x, y, scaleX, scaleY) {
  Widget.prototype.updatePosition.call(this, x, y, scaleX, scaleY)

  setTimeout(this.canvas.setSize.bind(this.canvas), 0)
}

Arm.prototype.getTouchedButton = function () {
  var x = this.pointer.x
  var y = this.pointer.y
  for (var i = 0; i < this.buttons.length; ++i) {
    var b = this.buttons[i]
    if (x > b.x && x < b.x + b.w && y > b.y && y < b.y + b.h) return b
  }
  return null
}

Arm.prototype.handleButton = function (text) {
  switch (text) {
    case 'RETRACT':
      if (this.animation !== null) break
      this.animation = new Animation(this)
      this.animation.addFrame(145, -35, 600)
      this.animation.addFrame(35, 19, 300)
      this.animation.start()
      break
    case 'EXTEND':
      if (this.animation !== null) break
      this.animation = new Animation(this)
      this.animation.addFrame(145, -35, 500)
      this.animation.addFrame(200, 18, 200)
      this.animation.addFrame(140, 79, 300)
      this.animation.start()
      break
    case 'GRAB':
      this.sendEvent('grab')
      break
  }
}

Arm.prototype.drawSegment = function (seg, color) {
  this.drawLine(seg.sx, seg.sy, seg.ex, seg.ey, color, 3, 6)
}

Arm.prototype.drawPointer = function (src, dst, color) {
  var ctx = this.canvas.ctx
  ctx.beginPath()
  ctx.strokeStyle = color
  ctx.fillStyle = color
  ctx.lineWidth = 2
  ctx.setLineDash([6, 3])
  ctx.moveTo(src.x, src.y)
  //  ctx.lineTo(dst.x, dst.y);
  ctx.moveTo(dst.x + this.TOUCH_TARGET_SIZE * 2, dst.y)
  ctx.arc(dst.x, dst.y, this.TOUCH_TARGET_SIZE * 2, 0, 2 * Math.PI)
  ctx.stroke()
  ctx.setLineDash([])
}

Arm.prototype.drawCircleDashed = function (x, y, radius, color) {
  var ctx = this.canvas.ctx
  ctx.beginPath()
  ctx.strokeStyle = color
  ctx.lineWidth = 2
  ctx.setLineDash([6, 3])
  ctx.moveTo(x, y)
  ctx.arc(x, y, radius, 0, 2 * Math.PI)
  ctx.stroke()
  ctx.setLineDash([])
}

Arm.prototype.drawTouchTarget = function (x, y) {
  var ctx = this.canvas.ctx
  ctx.beginPath()
  ctx.fillStyle = 'red'
  ctx.moveTo(x, y)
  ctx.arc(x, y, this.TOUCH_TARGET_SIZE, 0, 2 * Math.PI)
  ctx.fill()
}

Arm.prototype.drawLine = function (x0, y0, x1, y1, color, width, dotRadius) {
  var ctx = this.canvas.ctx
  ctx.beginPath()
  ctx.strokeStyle = color
  ctx.fillStyle = color
  ctx.lineWidth = width
  ctx.moveTo(x0, y0)
  ctx.lineTo(x1, y1)
  ctx.stroke()
  if (dotRadius !== undefined) {
    ctx.moveTo(x0, y0)
    ctx.arc(x0, y0, dotRadius, 0, 2 * Math.PI)
    ctx.moveTo(x1, y1)
    ctx.arc(x1, y1, dotRadius, 0, 2 * Math.PI)
    ctx.fill()
  }
}

Arm.prototype.updateAngles = function (updateAbsAngles) {
  var prev = null
  for (var i = 0; i < this.bones.length; ++i) {
    if (updateAbsAngles === true) this.bones[i].updatePos(prev, this.unit)
    prev = this.bones[i]
  }
}

Arm.prototype.run = function () {
  var dx = this.pointer.x - this.origin.x
  var dy = this.pointer.y - this.origin.y
  for (var i = 0; i < 10; ++i) {
    var res = this.solve(dx, dy)
    if (res == 0) {
      continue
    } else if (res == -1) {
      //console.log("FAILED");
    }
    break
  }

  this.updateAngles(true)
  this.draw()
}

Arm.prototype.getTargetPos = function () {
  if (this.bones === null || this.bones.length === 0) return null

  var end = this.bones[this.bones.length - 1]
  return {
    x: end.x / this.unit,
    y: end.y / this.unit
  }
}

Arm.prototype.draw = function () {
  var ctx = this.canvas.ctx

  ctx.fillStyle = '#ffffff'
  ctx.fillRect(0, 0, this.canvas.width, this.canvas.height)

  ctx.font = '12px monospace'
  ctx.fillStyle = 'black'
  var dx = (this.pointer.x - this.origin.x) / this.unit
  var dy = (this.pointer.y - this.origin.y) / this.unit
  ctx.fillText(dx.toFixed(1), 20, 12)
  ctx.fillText(dy.toFixed(1), 20, 24)

  var tgt = this.getTargetPos()
  if (tgt !== null) {
    ctx.fillText(tgt.x.toFixed(1), 80, 12)
    ctx.fillText(tgt.y.toFixed(1), 80, 24)
  }

  ctx.save()
  ctx.translate(this.origin.x, this.origin.y)
  var w = this.unit * this.BODY_RADIUS * 2
  var h = this.unit * this.BODY_HEIGHT

  ctx.fillStyle = '#C8A165'
  ctx.fillRect(-w / 2, this.ARM_BASE_HEIGHT * this.unit, w, h)

  for (var i = 0; i < this.bones.length; ++i) {
    var s = this.bones[i]

    ctx.save()
    ctx.rotate(s.relAngle)
    this.drawLine(0, 0, s.length * this.unit, 0, s.color, 3, 6)

    ctx.translate(s.length * this.unit, 0)
  }

  for (var i = 0; i < this.bones.length; ++i) {
    ctx.restore()
  }

  /*ctx.font = '18px monospace';
    ctx.fillStyle = "black"
    var y = this.BODY_HEIGHT*this.unit;
    for(var i = 0; i < this.bones.length; ++i) {
        var b = this.bones[i];

        ctx.fillText((b.angle >= 0 ? " " : "") + b.angle.toFixed(2), -100, y);
        ctx.fillText((b.angle >= 0 ? " " : "") + deg(b.angle).toFixed(2), 0, y);
        ctx.fillText((b.relAngle >= 0 ? " " : "") + b.relAngle.toFixed(2), 100, y);
        ctx.fillText((b.relAngle >= 0 ? " " : "") + deg(b.relAngle).toFixed(2), 200, y);

        var relBase = this.bones[i].angle - this.bones[0].angle;
        ctx.fillText((relBase >= 0 ? " " : "") + relBase.toFixed(2), -350, y);
        y += 20;
    }*/

  ctx.restore()

  ctx.strokeStyle = 'blue'
  ctx.lineWidth = 3
  ctx.textAlign = 'center'
  ctx.textBaseline = 'middle'
  ctx.font = this.buttons[0].h / 3 + 'px monospace'
  ctx.fillStyle = 'black'
  for (var i = 0; i < this.buttons.length; ++i) {
    var b = this.buttons[i]
    if (b.blink) {
      ctx.fillStyle = 'red'
      ctx.fillRect(b.x, b.y, b.w, b.h)
      ctx.fillStyle = 'black'
    }
    ctx.strokeRect(b.x, b.y, b.w, b.h)
    ctx.fillText(b.text, b.x + b.w / 2, b.y + b.h / 2)
  }

  this.drawPointer(this.origin, this.pointer, 'red')
}

Bone.prototype.rotate = function (prev, rotAng) {
  var newRelAng = clampAng(this.relAngle + rotAng)
  var _min = this.relMin
  var _max = this.relMax
  if (newRelAng < _min) {
    newRelAng = _min
  } else if (newRelAng > _max) {
    newRelAng = _max
  }

  var res = clampAng(newRelAng - this.relAngle)
  this.relAngle = newRelAng
  return res
}

Arm.prototype.fixBodyCollision = function () {
  var base = this.bones[0]
  var endBone = this.bones[this.bones.length - 1]
  base.relAngle = Math.min(Math.max(base.relAngle, base.relMin), base.relMax)
  this.updateAngles(true)

  while (this.isInBody(endBone.x, endBone.y)) {
    var newang = clampAng(base.relAngle - 0.01)
    if (newang > base.relMax || newang < base.relMin) return
    base.relAngle = newang
    this.updateAngles(true)
  }
}

Arm.prototype.isInBody = function (x, y) {
  return (
    Math.abs(x) <= this.BODY_RADIUS * this.unit &&
    y >= this.ARM_BASE_HEIGHT * this.unit
  )
}

Arm.prototype.solve = function (targetX, targetY) {
  if (this.bones === null || this.bones.length === 0) return

  var prev = null
  for (var i = 0; i < this.bones.length; ++i) {
    this.bones[i].updatePos(prev, this.unit)
    prev = this.bones[i]
  }

  // Limit under-robot positions
  if (targetX < this.unit * this.BODY_RADIUS) {
    if (targetY > this.unit * this.ARM_BASE_HEIGHT)
      targetY = this.unit * this.ARM_BASE_HEIGHT
  } else {
    if (targetY > this.unit * (this.ARM_BASE_HEIGHT + this.BODY_HEIGHT))
      targetY = this.unit * (this.ARM_BASE_HEIGHT + this.BODY_HEIGHT)
  }

  if (targetX < 5 * this.unit) {
    targetY = 0
    targetX = 0
  }

  var endX = prev.x
  var endY = prev.y

  var modifiedBones = false
  for (var i = this.bones.length - 1; i >= 0; --i) {
    var b = this.bones[i]

    var bx = 0
    var by = 0
    if (i > 0) {
      bx = this.bones[i - 1].x
      by = this.bones[i - 1].y
    }

    // Get the vector from the current bone to the end effector position.
    var curToEndX = endX - bx
    var curToEndY = endY - by
    var curToEndMag = Math.sqrt(curToEndX * curToEndX + curToEndY * curToEndY)

    // Get the vector from the current bone to the target position.
    var curToTargetX = targetX - bx
    var curToTargetY = targetY - by
    var curToTargetMag = Math.sqrt(
      curToTargetX * curToTargetX + curToTargetY * curToTargetY
    )

    // Get rotation to place the end effector on the line from the current
    // joint position to the target postion.
    var cosRotAng
    var sinRotAng
    var endTargetMag = curToEndMag * curToTargetMag

    if (endTargetMag <= 0.0001) {
      cosRotAng = 1
      sinRotAng = 0
    } else {
      cosRotAng =
        (curToEndX * curToTargetX + curToEndY * curToTargetY) / endTargetMag
      sinRotAng =
        (curToEndX * curToTargetY - curToEndY * curToTargetX) / endTargetMag
    }

    // Clamp the cosine into range when computing the angle (might be out of range
    // due to floating point error).
    var rotAng = Math.acos(Math.max(-1, Math.min(1, cosRotAng)))
    if (sinRotAng < 0.0) rotAng = -rotAng

    // Rotate the current bone in local space (this value is output to the user)
    // b.angle = overflow(b.angle + rotAng, Math.PI*2);
    rotAng = this.rotateArm(this.bones, i, rotAng)
    cosRotAng = Math.cos(rotAng)
    sinRotAng = Math.sin(rotAng)

    // Rotate the end effector position.
    endX = bx + cosRotAng * curToEndX - sinRotAng * curToEndY
    endY = by + sinRotAng * curToEndX + cosRotAng * curToEndY

    // Check for termination
    var endToTargetX = targetX - endX
    var endToTargetY = targetY - endY
    if (endToTargetX * endToTargetX + endToTargetY * endToTargetY <= 10) {
      this.fixBodyCollision()
      // We found a valid solution.
      return 1
    }

    // Track if the arc length that we moved the end effector was
    // a nontrivial distance.
    if (!modifiedBones && Math.abs(rotAng) * curToEndMag > 0.000001)
      modifiedBones = true
  }

  this.fixBodyCollision()

  if (modifiedBones) return 0
  return -1
}

Arm.prototype.rotateArm = function (bones, idx, rotAng) {
  var me = bones[idx]

  var base = bones[0]

  var newRelAng = clampAng(me.relAngle + rotAng)
  var _min = me.relMin
  var _max = me.relMax
  if (newRelAng < _min) {
    newRelAng = _min
  } else if (newRelAng > _max) {
    newRelAng = _max
  }

  var x = 0
  var y = 0
  var prevAng = 0
  for (var i = 0; i < bones.length; ++i) {
    var b = bones[i]
    var angle = b.relAngle
    if (idx == i) {
      angle = newRelAng
    }
    angle = clampAng(prevAng + angle)

    // arm helper-sticks collision with the bottom of the servo stand
    if (i == idx) {
      if (angle < b.absMin) {
        angle = b.absMin
        newRelAng = clampAng(angle - prevAng)
      } else if (angle > b.absMax) {
        angle = b.absMax
        newRelAng = clampAng(angle - prevAng)
      }
    }

    var nx = x + Math.cos(angle) * b.length * this.unit
    var ny = y + Math.sin(angle) * b.length * this.unit

    if (i > 0) {
      var diff = angle - base.angle
      if (diff < b.baseMin) {
        base.angle = clampAng(angle - b.baseMin)
      } else if (diff > b.baseMax) {
        base.angle = clampAng(angle - b.baseMax)
      }
    }

    x = nx
    y = ny
    prevAng = angle
  }

  var res = clampAng(newRelAng - me.relAngle)
  me.relAngle = newRelAng
  return res
}

Arm.prototype.update = function (diffMs) {
  if (!this.shouldSend()) return
  var pos = this.getTargetPos()
  if (pos !== null)
    this.sendEvent(
      'pos',
      {
        armX: pos.x,
        armY: pos.y
      },
      false
    )
}

function Bar(grid, uuid) {
  this.color = '#008000'
  this.fontSize = 16
  this.min = 0
  this.max = 100
  this.value = 0
  this.showValue = false

  var el = document.createElement('canvas')
  Widget.call(this, grid, uuid, Widget.wrapCanvas(el))

  this.w = 1
  this.h = 3

  this.canvas = ge1doot.canvas(el)
  this.canvas.resize = this.draw.bind(this)
}

Widget.createSubclass(Bar, {
  color: new Prop(String).setIsColor(),
  fontSize: new Prop(Number),
  min: new Prop(Number),
  max: new Prop(Number),
  value: new Prop(Number),
  showValue: new Prop(Boolean)
})

Bar.prototype.applyState = function (state) {
  Widget.prototype.applyState.call(this, state)
  this.draw()
}

Bar.prototype.updatePosition = function (x, y, scaleX, scaleY) {
  Widget.prototype.updatePosition.call(this, x, y, scaleX, scaleY)

  setTimeout(this.canvas.setSize.bind(this.canvas), 0)
}

Bar.prototype.draw = function () {
  var ctx = this.canvas.ctx
  ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

  ctx.save()

  var w = this.canvas.width
  var h = this.canvas.height
  if (this.h > this.w) {
    w = this.canvas.height
    h = this.canvas.width
    ctx.rotate(-Math.PI / 2)
    ctx.translate(-w, 0)
  }

  ctx.fillStyle = this.color
  ctx.strokeStyle = this.color
  ctx.lineWidth = 2

  var padding = 4
  ctx.strokeRect(padding, padding, w - padding * 2, h - padding * 2)

  var frac = (this.value - this.min) / (this.max - this.min)
  ctx.fillRect(
    padding * 2,
    padding * 2,
    (w - padding * 4) * frac,
    h - padding * 4
  )

  if (this.showValue) {
    ctx.translate(w / 2, h / 2)
    if (this.h > this.w) {
      ctx.rotate(Math.PI / 2)
    }
    ctx.lineWidth = 3
    ctx.strokeStyle = 'white'
    ctx.fillStyle = 'black'
    ctx.textAlign = 'center'
    ctx.textBaseline = 'middle'
    ctx.font = this.fontSize + 'px sans-serif'
    ctx.strokeText('' + this.value, 0, 0)
    ctx.fillText('' + this.value, 0, 0)
  }

  ctx.restore()
}

function Button(grid, uuid) {
  var el = document.createElement('button')
  el.style.display = 'flex'

  this.span = document.createElement('span')
  el.appendChild(this.span)
  this.span.innerText = 'Button'

  Widget.call(this, grid, uuid, el)

  this.w = 3
  this.h = 1

  this.pressed = false

  this.el.style.justifyContent = this.align = 'center'
  this.el.style.alignItems = this.valign = 'center'

  this.fontSize = 12
  this.span.style.fontSize = this.fontSize + 'pt'

  this.color = '#000000'
  this.background = ''

  if (!('ontouchstart' in document.documentElement)) {
    el.addEventListener(
      'mousedown',
      function (ev) {
        this.sendEvent('press')
        this.pressed = true
      }.bind(this)
    )

    document.addEventListener(
      'mouseup',
      function (ev) {
        if (!this.pressed) return
        this.sendEvent('release')
        this.pressed = false
      }.bind(this)
    )
  } else {
    this.el.addEventListener(
      'touchstart',
      function (ev) {        
        if (this.pressed) return
        this.pressed = true
        this.sendEvent('press', { pressed: true })
      }.bind(this)
    )
    this.el.addEventListener(
      'touchend',
      function (ev) {
        if (!this.pressed) return
        this.pressed = false
        this.sendEvent('release', { pressed: false })
      }.bind(this)
    )
  }
}

Widget.createSubclass(Button, {
  text: new Prop(
    String,
    function () {
      return this.span.innerText
    },
    function (val) {
      this.span.innerText = val
    }
  ),
  fontSize: new Prop(Number, undefined, function (val) {
    this.fontSize = val
    this.span.style.fontSize = val + 'pt'
  }),
  color: new Prop(String, undefined, function (val) {
    this.color = val
    this.el.style.color = val
  }).setIsColor(),
  background: new Prop(String, undefined, function (val) {
    this.background = val
    this.el.style.backgroundColor = val
  }).setIsColor(),
  align: new Prop(String, undefined, function (val) {
    this.align = val
    this.el.style.justifyContent = val
  }).setOptions(['flex-start', 'center', 'flex-end']),
  valign: new Prop(String, undefined, function (val) {
    this.valign = val
    this.el.style.alignItems = val
  }).setOptions(['flex-start', 'center', 'flex-end']),
  disabled: new Prop(
    Boolean,
    function () {
      return this.el.disabled
    },
    function (val) {
      this.el.disabled = !!val
    }
  )
})

function Camera(grid, uuid) {
  var el = document.createElement('canvas')
  Widget.call(this, grid, uuid, Widget.wrapCanvas(el))

  this.w = 4
  this.h = 4
  this.rotation = 90
  this.clip = false
  this.frame = null
  this.badStatus = false
  this.tags = []

  this.canvas = ge1doot.canvas(el)
  this.canvas.resize = this.draw.bind(this)

  if(window['IN_RB_GRID_DESIGNER'] !== true) {
    this.scheduleFrameDownload();
  }
}

Widget.createSubclass(Camera, {
  rotation: new Prop(Number),
  clip: new Prop(Boolean),
  tags: new Prop(Array, function() {
    return this.tags
  }, function(tags) {
    this.tags = tags
  }).disableEdit(),
})

Camera.prototype.MIN_LIBRARY_VERSION = 0x041000

Camera.prototype.updatePosition = function (x, y, scaleX, scaleY) {
  Widget.prototype.updatePosition.call(this, x, y, scaleX, scaleY)

  setTimeout(this.canvas.setSize.bind(this.canvas), 0)
}

Camera.prototype.applyState = function (state) {
  Widget.prototype.applyState.call(this, state)
  this.draw()
}

Camera.prototype.scheduleFrameDownload = function() {
  requestAnimationFrame(this.downloadFrame.bind(this))
}

Camera.prototype.downloadFrame = function() {
  this.badStatus = false

  var req = new XMLHttpRequest();
  req.responseType = 'blob';
  req.timeout = 500

  req.addEventListener("load", function() {
    if(req.status !== 200) {
      this.badStatus = true
      this.draw()
      setTimeout(this.scheduleFrameDownload.bind(this), req.status === 503 ? 500 : 10000)
      return;
    }

    createImageBitmap(req.response)
      .then(function(bitmap) {
        if(this.frame !== null) {
          this.frame.close()
        }
        this.frame = bitmap
        this.draw()
      }.bind(this))
  }.bind(this));

  req.addEventListener("loadend", function() {
    if(!this.badStatus) {
      this.scheduleFrameDownload()
    }
  }.bind(this))
  
  req.open("GET", "/extra/camera.jpg");
  req.send();
}

Camera.prototype.draw = function() {
  var ctx = this.canvas.ctx
  var cw = this.canvas.width
  var ch = this.canvas.height

  var fw, fh
  if(this.frame !== null) {
    fw = this.frame.width
    fh = this.frame.height
  } else {
    fw = 4
    fh = 3
  }

  ctx.clearRect(0, 0, cw, ch)
  ctx.save();
  ctx.translate(cw/2,ch/2);

  // swap dimensions on rotation
  if(this.rotation%180 >= 45 && this.rotation%180 < 135) {
    var t = cw
    cw = ch
    ch = t
  }

  var ratioCanvas = cw / ch
  var rationFrame = fw / fh

  var scaledW, scaledH
  if(!this.clip) {
    if(rationFrame > ratioCanvas) {
      scaledW = cw
      scaledH = cw / rationFrame
    } else {
      scaledH = ch
      scaledW = ch * rationFrame
    }
  } else {
    if(rationFrame > ratioCanvas) {
      scaledH = ch
      scaledW = ch * rationFrame
    } else {
      scaledW = cw
      scaledH = cw / rationFrame
    }
  }

  if(this.frame !== null) {
    ctx.rotate(this.rotation*Math.PI/180);
    ctx.drawImage(this.frame, -scaledW/2, -scaledH/2, scaledW, scaledH)
  } else {
    ctx.font = "16px serif";
    ctx.textAlign = "center"
    if(this.badStatus) {
      ctx.fillText("No camera present.", 0, 0, Math.min(cw, ch))
    } else {
      ctx.fillText("No frame yet.", 0, 0, Math.min(cw, ch))
    }

    ctx.rotate(this.rotation*Math.PI/180);
    ctx.strokeStyle = "black";
    ctx.strokeRect(-scaledW/2, -scaledH/2, scaledW, scaledH)
  }

  var scale = scaledW / fw

  ctx.font = "40px sans-serif";
  ctx.fillStyle = "red";
  ctx.textAlign = "center"
  ctx.strokeStyle = "red";
  ctx.lineWidth = 3;

  for(var i = 0; i < this.tags.length; ++i) {
    var t = this.tags[i];

    ctx.beginPath();
    ctx.moveTo(t.c00*scale - scaledW/2, t.c01*scale - scaledH/2)
    ctx.lineTo(t.c10*scale - scaledW/2, t.c11*scale - scaledH/2)
    ctx.lineTo(t.c20*scale - scaledW/2, t.c21*scale - scaledH/2)
    ctx.lineTo(t.c30*scale - scaledW/2, t.c31*scale - scaledH/2)
    ctx.lineTo(t.c00*scale - scaledW/2, t.c01*scale - scaledH/2)
    ctx.stroke();

    var textX = ((t.c00 + t.c20)/2)*scale - scaledW/2
    var textY = ((t.c01 + t.c21)/2)*scale - scaledH/2
    ctx.save()
    ctx.translate(textX, textY)
    ctx.rotate(this.rotation*Math.PI/180*-1)
    ctx.fillText(t.id.toString(), 0, 0)
    ctx.restore()
  }
  
  ctx.restore()
}

function Checkbox(grid, uuid) {
  this.color = '#000000'
  this.checked = false
  this.text = 'CheckBox'
  this.fontSize = 14

  var el = document.createElement('canvas')
  Widget.call(this, grid, uuid, Widget.wrapCanvas(el))

  this.w = 4
  this.h = 1

  this.canvas = ge1doot.canvas(el)
  this.canvas.resize = this.draw.bind(this)
  this.canvas.pointer.up = function () {
    this.checked = !this.checked
    this.draw()
    this.sendEvent('checked', { checked: this.checked })
  }.bind(this)
}

Widget.createSubclass(Checkbox, {
  fontSize: new Prop(Number),
  checked: new Prop(Boolean),
  color: new Prop(String).setIsColor(),
  text: new Prop(String)
})

Checkbox.prototype.applyState = function (state) {
  Widget.prototype.applyState.call(this, state)
  this.draw()
}

Checkbox.prototype.updatePosition = function (x, y, scaleX, scaleY) {
  Widget.prototype.updatePosition.call(this, x, y, scaleX, scaleY)
  setTimeout(this.canvas.setSize.bind(this.canvas), 0)
}

Checkbox.prototype.draw = function () {
  var ctx = this.canvas.ctx
  ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

  ctx.save()

  var w = this.canvas.width
  var h = this.canvas.height
  if (this.h > this.w) {
    w = this.canvas.height
    h = this.canvas.width
    ctx.rotate(Math.PI / 2)
    ctx.translate(0, -h)
  }

  var side = Math.min(w, h)
  var padding = side * 0.1
  side -= padding * 2

  if (this.text !== '') {
    var textX = side + padding * 3
    ctx.textBaseline = 'middle'
    ctx.font = this.fontSize + 'px sans-serif'
    ctx.fillText(this.text, textX, h / 2, w - textX)
    ctx.translate(padding + side / 2, padding + side / 2)
  } else {
    ctx.translate(w / 2, h / 2)
  }

  ctx.fillStyle = this.color
  ctx.strokeStyle = this.color
  ctx.lineWidth = 2

  ctx.strokeRect(-side / 2, -side / 2, side, side)
  if (this.checked) {
    ctx.fillRect(
      -side / 2 + padding,
      -side / 2 + padding,
      side - padding * 2,
      side - padding * 2
    )
  }

  ctx.restore()
}

function Circle(grid, uuid) {
  this.color = '#008000'
  this.fontSize = 16
  this.min = 0
  this.max = 360
  this.valueStart = 0
  this.value = 200
  this.showValue = true
  this.lineWidth = 10

  var el = document.createElement('canvas')
  Widget.call(this, grid, uuid, Widget.wrapCanvas(el))

  this.w = 3
  this.h = 3

  this.canvas = ge1doot.canvas(el)
  this.canvas.resize = this.draw.bind(this)
}

Widget.createSubclass(Circle, {
  color: new Prop(String).setIsColor(),
  fontSize: new Prop(Number),
  min: new Prop(Number),
  max: new Prop(Number),
  lineWidth: new Prop(Number),
  valueStart: new Prop(Number),
  value: new Prop(Number),
  showValue: new Prop(Boolean)
})

Circle.prototype.MIN_LIBRARY_VERSION = 0x040700

Circle.prototype.applyState = function (state) {
  Widget.prototype.applyState.call(this, state)
  this.draw()
}

Circle.prototype.updatePosition = function (x, y, scaleX, scaleY) {
  Widget.prototype.updatePosition.call(this, x, y, scaleX, scaleY)
  setTimeout(this.canvas.setSize.bind(this.canvas), 0)
}

Circle.prototype.draw = function () {
  var ctx = this.canvas.ctx
  ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

  ctx.save()

  var padding = 4 + this.lineWidth / 2
  var w = this.canvas.width
  var h = this.canvas.height
  var arcRadius = Math.max(1, Math.min(w, h) / 2 - padding)

  if (!w || !h) {
    return
  }

  var valueStartClipped = Math.max(this.valueStart, this.min)
  var angleStart =
    ((valueStartClipped - this.min) / (this.max - this.min)) * Math.PI * 2
  var angleEnd = ((this.value - this.min) / (this.max - this.min)) * Math.PI * 2
  // Let's start on top
  angleStart -= Math.PI / 2
  angleEnd -= Math.PI / 2

  ctx.lineWidth = this.lineWidth

  ctx.beginPath()
  ctx.globalAlpha = 0.1
  ctx.strokeStyle = this.color
  ctx.arc(w / 2, h / 2, arcRadius, 0, Math.PI * 2)
  ctx.stroke()

  ctx.globalAlpha = 1
  ctx.beginPath()
  ctx.strokeStyle = this.color
  ctx.arc(w / 2, h / 2, arcRadius, angleStart, angleEnd)
  ctx.stroke()

  if (this.showValue) {
    ctx.translate(w / 2, h / 2)
    ctx.lineWidth = 3
    ctx.strokeStyle = 'white'
    ctx.fillStyle = 'black'
    ctx.textAlign = 'center'
    ctx.textBaseline = 'middle'
    ctx.font = this.fontSize + 'px sans-serif'
    ctx.strokeText('' + this.value, 0, 0)
    ctx.fillText('' + this.value, 0, 0)
  }

  ctx.restore()
}

function Input(grid, uuid) {
  var el = document.createElement('div')
  el.style.padding = '2px'

  this.input = document.createElement('input')
  this.input.type = 'text'
  this.input.style.all = 'revert'
  this.input.style.width = '100%'
  this.input.style.height = '100%'
  this.input.style.borderWidth = '2px'
  this.input.style.boxSizing = 'border-box'
  this.input.style.borderColor = this.color = '#cccccc'
  el.appendChild(this.input)

  Widget.call(this, grid, uuid, el)

  this.w = 4
  this.h = 1

  this.input.addEventListener(
    'change',
    function () {
      this.sendEvent('changed', { text: this.input.value })
    }.bind(this)
  )

  if ('ontouchstart' in document.documentElement) {
    this.input.addEventListener(
      'focus',
      function () {
        if (this.input.parentElement === this.el) {
          this.el.removeChild(this.input)
          this.el.parentElement.appendChild(this.input)
          this.input.style.boxShadow = '5px 5px 5px black'
          this.input.style.fontSize = '150%'
          this.input.style.position = 'absolute'
          this.input.style.top = '15px'
          this.input.style.left = '50%'
          this.input.style.marginLeft = '-45%'
          this.input.style.width = '90%'
          this.input.style.height = '2em'
          this.input.focus()
        }
      }.bind(this)
    )

    this.input.addEventListener(
      'blur',
      function () {
        if (this.input.parentElement !== this.el) {
          this.el.parentElement.removeChild(this.input)
          this.el.appendChild(this.input)
          this.input.style.width = '100%'
          this.input.style.height = '100%'
          this.input.style.boxShadow = null
          this.input.style.position = null
          this.input.style.top = null
          this.input.style.left = null
          this.input.style.marginLeft = null
          this.input.style.fontSize = null
        }
      }.bind(this)
    )
  }
}

Widget.createSubclass(Input, {
  text: new Prop(
    String,
    function () {
      return this.input.value
    },
    function (val) {
      this.input.value = val
    }
  ),
  color: new Prop(String, undefined, function (val) {
    this.color = val
    this.input.style.borderColor = val
  }).setIsColor(),
  type: new Prop(
    String,
    function () {
      return this.input.type
    },
    function (val) {
      this.input.type = val
    }
  ).setOptions(['text', 'number', 'password']),
  disabled: new Prop(
    Boolean,
    function () {
      return this.input.disabled
    },
    function (val) {
      this.input.disabled = !!val
    }
  )
})

function Joystick(grid, uuid) {
  var el = document.createElement('div')

  this.color = '#FF0000'
  this.text = ''
  this.keys = ''

  Widget.call(this, grid, uuid, el)

  this.w = 4
  this.h = 4

  this.radius = 0
  this.valX = 0
  this.valY = 0
  this.buttonClickHandler = null
  this.touchStart = null
  this.pressedKeys = {}
  this.manager = null

  document.addEventListener('keydown', this.onKeyDown.bind(this))
  document.addEventListener('keyup', this.onKeyUp.bind(this))
}

Widget.createSubclass(Joystick, {
  color: new Prop(String).setIsColor(),
  keys: new Prop(String),
  text: new Prop(String)
})

Joystick.prototype.applyState = function (state) {
  Widget.prototype.applyState.call(this, state)

  if (this.manager !== null) this.createNippleJs()
}

Joystick.prototype.updatePosition = function (x, y, scaleX, scaleY) {
  Widget.prototype.updatePosition.call(this, x, y, scaleX, scaleY)

  setTimeout(this.createNippleJs.bind(this), 0)
}

Joystick.prototype.createNippleJs = function () {
  if (this.manager !== null) {
    this.manager.destroy()
  }

  this.radius = Math.min(this.el.clientHeight, this.el.clientWidth) / 2

  this.manager = nipplejs.create({
    zone: this.el,
    mode: 'static',
    color: this.color,
    size: this.radius * 2,
    position: {
      top: '50%',
      left: '50%'
    },
    restOpacity: 0.9,
    fadeTime: 0
  })

  this.joy = this.manager.get(this.manager.ids[0])

  if (this.text !== '') {
    var nipple = this.joy.ui.front
    nipple.innerHTML = this.text
    nipple.style.fontWeight = 'bold'
    nipple.style.fontFamily = 'Arial, Helvetica, sans-serif'
    nipple.style.color = 'white'
    nipple.style.textAlign = 'center'
    nipple.style.verticalAlign = 'middle'
    nipple.style.lineHeight = nipple.style.height
  }

  this.manager.on(
    'start',
    function (event, data) {
      this.touchStart = Date.now()
      this.valX = 0
      this.valY = 0
    }.bind(this)
  )

  this.manager.on(
    'move',
    function (event, data) {
      var dist = (data.distance / this.radius) * 32767
      this.valX = (Math.cos(data.angle.radian) * dist) | 0
      this.valY = (Math.sin(data.angle.radian) * dist) | 0
    }.bind(this)
  )

  this.manager.on(
    'end',
    function (event, data) {
      var diff = Date.now() - this.touchStart
      if (
        diff < 150 &&
        Math.abs(this.valX) < 8000 &&
        Math.abs(this.valY) < 8000
      ) {
        this.sendEvent('click')
      }

      this.valX = 0
      this.valY = 0
    }.bind(this)
  )
}

Joystick.prototype.onKeyDown = function (ev) {
  if (this.keys === '') return
  var idx = this.keys.indexOf(ev.key)
  if (idx === -1) return
  if (idx === 4) {
    this.sendEvent('click')
  }
  this.pressedKeys[idx] = true
  this.updateNippleFront()
}

Joystick.prototype.onKeyUp = function (ev) {
  if (this.keys === '') return
  var idx = this.keys.indexOf(ev.key)
  if (idx === -1) return
  this.pressedKeys[idx] = false
  this.updateNippleFront()
}

Joystick.prototype.updateNippleFront = function () {
  var p = this.pressedKeys
  var left = 0
  var top = 0
  if (p[0] === true) top -= 1
  if (p[2] === true) top += 1
  if (p[1] === true) left -= 1
  if (p[3] === true) left += 1

  this.valX = left * 28000
  this.valY = top * -28000

  var fr = this.joy.ui.front
  fr.style.left = left * this.radius + 'px'
  fr.style.top = top * this.radius + 'px'
}

Joystick.prototype.update = function () {
  this.sendEvent(
    'pos',
    {
      jx: this.valX,
      jy: this.valY
    },
    false
  )
}

function Led(grid, uuid) {
  this.color = '#FF0000'
  this.on = false

  var el = document.createElement('canvas')

  Widget.call(this, grid, uuid, Widget.wrapCanvas(el))

  this.w = 1
  this.h = 1

  this.canvas = ge1doot.canvas(el)
  this.canvas.resize = this.draw.bind(this)
}

Widget.createSubclass(Led, {
  color: new Prop(String).setIsColor(),
  on: new Prop(Boolean)
})

Led.prototype.applyState = function (state) {
  Widget.prototype.applyState.call(this, state)
  this.draw()
}

Led.prototype.updatePosition = function (x, y, scaleX, scaleY) {
  Widget.prototype.updatePosition.call(this, x, y, scaleX, scaleY)

  setTimeout(this.canvas.setSize.bind(this.canvas), 0)
}

Led.prototype.draw = function () {
  var ctx = this.canvas.ctx
  ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

  var x = this.canvas.width / 2
  var y = this.canvas.height / 2
  var radius = Math.min(this.canvas.height, this.canvas.width) / 2

  ctx.save()
  if (this.on) {
    ctx.fillStyle = this.color
    ctx.shadowColor = this.color
    ctx.shadowBlur = radius * 0.5
    ctx.lineWidth = 1
    ctx.beginPath()
    ctx.arc(x, y, radius * 0.5, 0, Math.PI * 2, false)
    ctx.fill()
  } else {
    ctx.lineWidth = radius * 0.1
    ctx.strokeStyle = this.color

    ctx.beginPath()
    ctx.arc(x, y, radius * 0.5, 0, Math.PI * 2, false)
    ctx.stroke()
  }
  ctx.restore()
}

function Orientation(grid, uuid) {
  this.color = '#FF0000'

  if (
    window['RbGravitySensor'] === undefined &&
    window['IN_RB_GRID_DESIGNER'] !== true
  ) {
    this.enabled = false
    this.canvas = null

    var el = document.createElement('span')
    el.innerText =
      'Orientation sensor requires the Android RBController app, version >= 1.9.'
    Widget.call(this, grid, uuid, el)
  } else {
    this.enabled = window['IN_RB_GRID_DESIGNER'] !== true

    var el = document.createElement('canvas')
    Widget.call(this, grid, uuid, Widget.wrapCanvas(el))
    this.canvas = ge1doot.canvas(el)
    this.canvas.resize = this.draw.bind(this)

    if (this.enabled) {
      RbGravitySensor.start()
    } else {
      this.demoRollDelta = 0.02
      requestAnimationFrame(this.doGridDesignerDemo.bind(this))
    }
  }

  this.w = 1
  this.h = 1

  this.yaw = 0.0
  this.pitch = 0.0
  this.roll = 0.0
}
Widget.createSubclass(Orientation, {
  color: new Prop(String).setIsColor()
})

Orientation.prototype.MIN_LIBRARY_VERSION = 0x040200

Orientation.prototype.applyState = function (state) {
  Widget.prototype.applyState.call(this, state)
  this.draw()
}

Orientation.prototype.updatePosition = function (x, y, scaleX, scaleY) {
  Widget.prototype.updatePosition.call(this, x, y, scaleX, scaleY)

  if (this.canvas !== null) setTimeout(this.canvas.setSize.bind(this.canvas), 0)
}

Orientation.prototype.doGridDesignerDemo = function () {
  if (!this.canvas.elem.isConnected) {
    return
  }
  this.roll += this.demoRollDelta
  if (this.roll > 1 || this.roll < -1) {
    this.demoRollDelta *= -1
  }
  this.draw()
  requestAnimationFrame(this.doGridDesignerDemo.bind(this))
}

Orientation.prototype.draw = function () {
  if (this.canvas === null) return

  var ctx = this.canvas.ctx
  ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

  var radius = Math.min(this.canvas.height, this.canvas.width) / 4
  var centerX = this.canvas.width / 2
  var centerY = this.canvas.height / 2

  var y = centerY + this.yaw * (this.canvas.height / 2)
  var x = centerX - this.roll * (this.canvas.height / 2)

  ctx.save()
  ctx.fillStyle = this.color
  ctx.shadowColor = this.color
  ctx.shadowBlur = radius * 0.5
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.arc(x, y, radius * 0.5, 0, Math.PI * 2, false)
  ctx.fill()
  ctx.restore()

  ctx.strokeColor = '#000'
  ctx.beginPath()
  ctx.arc(centerX, centerY, radius * 0.5, 0, Math.PI * 2, false)
  ctx.stroke()
}

Orientation.prototype.update = function () {
  if (this.enabled === false) {
    return
  }

  this.yaw = RbGravitySensor.getYaw()
  this.pitch = RbGravitySensor.getPitch()
  this.roll = RbGravitySensor.getRoll()

  this.sendEvent(
    'pos',
    {
      oy: this.yaw,
      op: this.pitch,
      or: this.roll
    },
    false
  )
  this.draw()
}

function Select(grid, uuid) {
  var el = document.createElement('select')
  el.style.display = 'flex'

  Widget.call(this, grid, uuid, el)

  this.w = 3
  this.h = 1

  this.el.selectedIndex = this.selectedIndex = 0
  this.options = "None"
  this.el.add(new Option("None", "0"))

  this.color = '#000000'
  this.background = ''

  this.el.addEventListener(
    'change',
    function () {
      this.sendEvent('changed', { value: this.selectedIndex })
    }.bind(this)
  )
}

Widget.createSubclass(Select, {
  color: new Prop(String, undefined, function (val) {
    this.color = val
    this.el.style.color = val
  }).setIsColor(),
  background: new Prop(String, undefined, function (val) {
    this.background = val
    this.el.style.backgroundColor = val
  }).setIsColor(),
  disabled: new Prop(
    Boolean,
    function () {
      return this.el.disabled
    },
    function (val) {
      this.el.disabled = !!val
    }
  ),
  options: new Prop(
    String,
    function () {
      const opts = Array(this.el.length)

      for (let i = 0; i < this.el.length; i++) {
        opts[this.el[i].value] = this.el[i].text
      }

      return opts.join(',')
    },

    function (val) {
      if (!val) val = "None"
      while (this.el.length > 0) {
        this.el.remove(0)
      }

      const newOpts = val.split(",")
      for (let index = 0; index < newOpts.length; index++) {
        this.el.add(new Option(newOpts[index], index))
      }

      this.options = val
    }
  ),
  selectedIndex: new Prop(
    Number,
    function () {
      return this.el.selectedIndex
    },
    function (val) {
      this.el.selectedIndex = val
    }
  ),
})

Select.prototype.MIN_LIBRARY_VERSION = 0x040800

function Slider(grid, uuid) {
  this.PADDING_FRAC = 0.03

  this.color = '#008000'
  this.fontSize = 16
  this.min = 0
  this.max = 100
  this.value = 0
  this.showValue = true
  this.precision = 1

  var el = document.createElement('canvas')
  Widget.call(this, grid, uuid, Widget.wrapCanvas(el))

  this.w = 4
  this.h = 1
  this.touched = false
  this.sendValueTimer = null

  this.canvas = ge1doot.canvas(el)
  this.canvas.resize = this.draw.bind(this)

  var pointer = this.canvas.pointer

  pointer.down = function () {
    this.touched = true
    this.movePos(pointer)
  }.bind(this)

  pointer.up = function () {
    this.touched = false
  }.bind(this)

  pointer.move = function () {
    if (this.touched) {
      this.movePos(pointer)
    }
  }.bind(this)
}

Widget.createSubclass(Slider, {
  color: new Prop(String).setIsColor(),
  fontSize: new Prop(Number),
  min: new Prop(Number),
  max: new Prop(Number),
  value: new Prop(Number),
  precision: new Prop(Number),
  showValue: new Prop(Boolean)
})

Slider.prototype.applyState = function (state) {
  Widget.prototype.applyState.call(this, state)
  this.draw()
}

Slider.prototype.updatePosition = function (x, y, scaleX, scaleY) {
  Widget.prototype.updatePosition.call(this, x, y, scaleX, scaleY)

  setTimeout(this.canvas.setSize.bind(this.canvas), 0)
}

Slider.prototype.roundToPrecision = function (x, precision) {
  var y = +x + (precision === undefined ? 0.5 : precision / 2)
  return y - (y % (precision === undefined ? 1 : +precision))
}

Slider.prototype.movePos = function (pointer) {
  var x = pointer.x
  var w = this.canvas.width
  if (this.h > this.w) {
    x = this.canvas.height - pointer.y
    w = this.canvas.height
  }

  var padding = w * this.PADDING_FRAC
  x = Math.min(w - padding, x)
  x = Math.max(0, x - padding)
  w -= padding * 2

  var frac = x / w
  this.value = this.roundToPrecision(
    this.min + (this.max - this.min) * frac,
    this.precision
  )
  this.sendValue()
  this.draw()
}

Slider.prototype.sendValue = function () {
  if (this.sendValueTimer !== null) return

  this.sendValueTimer = setTimeout(
    function () {
      this.sendValueTimer = null
      this.sendEvent('changed', { value: this.value })
    }.bind(this),
    100
  )
}

Slider.prototype.formatValue = function (value, precision) {
  var pStr = '' + precision
  var idx = pStr.indexOf('.')
  var decimals = 0
  if (idx !== -1) {
    decimals = pStr.length - (idx + 1)
  }
  return value.toFixed(decimals)
}

Slider.prototype.draw = function () {
  var ctx = this.canvas.ctx
  ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

  ctx.save()

  var w = this.canvas.width
  var h = this.canvas.height
  if (this.h > this.w) {
    w = this.canvas.height
    h = this.canvas.width
    ctx.rotate(-Math.PI / 2)
    ctx.translate(-w, 0)
  }

  ctx.fillStyle = this.color
  ctx.strokeStyle = this.color
  ctx.lineWidth = 2

  var padding = w * this.PADDING_FRAC

  var guideH = Math.max(2, h * 0.05)
  ctx.fillRect(padding, h / 2 - guideH / 2, w - padding * 2, guideH)

  var frac = (this.value - this.min) / (this.max - this.min)
  guideH *= 2
  ctx.fillRect(padding + (w - padding * 2) * frac - guideH / 2, 0, guideH, h)

  if (this.showValue) {
    ctx.translate(w / 2, h / 2)
    if (this.h > this.w) {
      ctx.rotate(Math.PI / 2)
    }
    ctx.lineWidth = 3
    ctx.strokeStyle = 'white'
    ctx.fillStyle = 'black'
    ctx.textAlign = 'center'
    ctx.textBaseline = 'middle'
    ctx.font = this.fontSize + 'px sans-serif'
    var txt = this.formatValue(this.value, this.precision)
    ctx.strokeText(txt, 0, 0)
    ctx.fillText(txt, 0, 0)
  }

  ctx.restore()
}

function SpinEdit(grid, uuid) {
  this.color = '#000000'
  this.fontSize = 14
  this.value = 0
  this.step = 1
  this.precision = 1

  var el = document.createElement('canvas')
  Widget.call(this, grid, uuid, Widget.wrapCanvas(el))

  this.w = 4
  this.h = 1

  this.canvas = ge1doot.canvas(el)
  this.canvas.resize = this.draw.bind(this)
  this.canvas.pointer.up = function () {
    const part = this.canvas.width / 4
    if (this.canvas.pointer.x < part) {
      this.value -= this.step
    } else if (this.canvas.pointer.x > part * 3) {
      this.value += this.step
    } else {
      return
    }

    this.sendEvent('changed', { value: this.value })
    this.draw()
  }.bind(this)
}

Widget.createSubclass(SpinEdit, {
  fontSize: new Prop(Number),
  color: new Prop(String).setIsColor(),
  value: new Prop(Number),
  step: new Prop(Number),
  precision: new Prop(Number)
})

SpinEdit.prototype.MIN_LIBRARY_VERSION = 0x040600

SpinEdit.prototype.applyState = function (state) {
  Widget.prototype.applyState.call(this, state)
  this.draw()
}

SpinEdit.prototype.updatePosition = function (x, y, scaleX, scaleY) {
  Widget.prototype.updatePosition.call(this, x, y, scaleX, scaleY)

  setTimeout(this.canvas.setSize.bind(this.canvas), 0)
}

SpinEdit.prototype.formatValue = function (value, precision) {
  var pStr = '' + precision
  var idx = pStr.indexOf('.')
  var decimals = 0
  if (idx !== -1) {
    decimals = pStr.length - (idx + 1)
  }
  return value.toFixed(decimals)
}

SpinEdit.prototype.draw = function () {
  var ctx = this.canvas.ctx
  ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

  const part = this.canvas.width / 4
  const padding = part * 0.2

  ctx.lineWidth = Math.max(2, this.canvas.height * 0.05)
  ctx.fillStyle = this.color
  ctx.strokeStyle = this.color

  // minus
  ctx.strokeRect(padding, this.canvas.height / 2, part - padding * 2, 1)

  // plus
  ctx.save()
  ctx.translate(part * 3, 0)
  const minusW = part - padding * 2
  ctx.strokeRect(padding, this.canvas.height / 2, minusW, 1)
  ctx.strokeRect(part / 2, this.canvas.height / 2 - minusW / 2, 1, minusW)
  ctx.restore()

  // text
  ctx.save()
  ctx.textAlign = 'center'
  ctx.textBaseline = 'middle'
  ctx.font = this.fontSize + 'px sans-serif'
  ctx.fillText(
    this.formatValue(this.value, this.precision),
    this.canvas.width / 2,
    this.canvas.height / 2
  )
  ctx.restore()
}

function Switcher(grid, uuid) {
  this.color = '#000000'
  this.fontSize = 14
  this.value = 0
  this.__switcher = true
  this.grid = grid

  var el = document.createElement('div')
  Widget.call(this, grid, uuid, el)

  var canvas = document.createElement('canvas')
  canvas.style.width = '100%'
  canvas.style.height = '100%'

  this.input = document.createElement('input')
  this.input.type = 'text'
  this.input.style.all = 'revert'
  this.input.style.height = '100%'
  this.input.style.borderWidth = '2px'
  this.input.style.boxSizing = 'border-box'
  this.input.style.borderColor = '#cccccc'
  this.input.style.position = 'absolute'

  el.appendChild(canvas)
  el.appendChild(this.input)

  this.w = 4
  this.h = 1

  this.canvas = ge1doot.canvas(canvas)
  this.canvas.resize = this.draw.bind(this)
  this.canvas.pointer.up = function () {
    var bounds = this.el.getBoundingClientRect()
    const width = bounds.width
    const height = bounds.height
    const part = width >= 3 * height
      ? height
      : this.canvas.width / 4
    const inputWidth = width >= 3 * height
      ? (width - 2 * height)
      : width / 2
    if (this.canvas.pointer.x < part) {
      this.value -= 1
      if (this.value < this.min)
        this.value = this.min
    } else if (this.canvas.pointer.x > part + inputWidth) {
      this.value += 1
      if (this.value > this.max)
        this.value = this.max
    } else {
      return
    }

    this.sendEvent('changed', { value: this.value })
    this.draw()
  }.bind(this)

  this.input.addEventListener(
    'change',
    function () {
      const val = parseInt(this.input.value, 10)
      if (val < this.min)
        val = this.min
      else if (val > min(this.max, this.grid.tabs.length))
        val = min(this.max, this.grid.tabs.length)
      this.value = val
      this.draw()
      this.sendEvent('changed', { value: this.value })
    }.bind(this)
  )
}

Widget.createSubclass(Switcher, {
  fontSize: new Prop(Number),
  color: new Prop(String).setIsColor(),
  value: new Prop(Number),
  min: new Prop(Number),
  max: new Prop(Number)
})

Switcher.prototype.MIN_LIBRARY_VERSION = 0x041000

Switcher.prototype.applyState = function (state) {
  Widget.prototype.applyState.call(this, state)
  this.draw()
}

Switcher.prototype.updatePosition = function (x, y, scaleX, scaleY) {
  Widget.prototype.updatePosition.call(this, x, y, scaleX, scaleY)

  setTimeout(this.canvas.setSize.bind(this.canvas), 0)
}

Switcher.prototype.formatValue = function (value, precision) {
  var pStr = '' + precision
  var idx = pStr.indexOf('.')
  var decimals = 0
  if (idx !== -1) {
    decimals = pStr.length - (idx + 1)
  }
  return value.toFixed(decimals)
}

Switcher.prototype.draw = function () {
  var bounds = this.el.getBoundingClientRect()
  const width = bounds.width
  const height = bounds.height

  var ctx = this.canvas.ctx

  const inputWidth = width >= 3 * height
    ? (width - 2 * height)
    : width / 2

  const part = width >= 3 * height
    ? height
    : this.canvas.width / 4

  let padding = part * 0.2

  this.input.style.width = inputWidth + 'px'
  this.input.style.left = part + 'px'
  this.input.value = this.value.toString()

  ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

  ctx.lineWidth = Math.max(2, this.canvas.height * 0.05)
  ctx.fillStyle = this.color
  ctx.strokeStyle = this.color

  // left
  ctx.save()
  ctx.translate(padding, this.canvas.height / 2)
  ctx.beginPath()
  ctx.moveTo(0, 0)
  ctx.lineTo(part - padding * 2, this.canvas.height / 4)
  ctx.moveTo(0, 0)
  ctx.lineTo(part - padding * 2, - this.canvas.height / 4)
  ctx.stroke()
  ctx.restore()

  // right
  ctx.save()
  ctx.translate(this.canvas.width - padding, this.canvas.height / 2)
  ctx.beginPath()
  ctx.moveTo(padding * 2 - part, this.canvas.height / 4)
  ctx.lineTo(0, 0)
  ctx.moveTo(padding * 2 - part, - this.canvas.height / 4)
  ctx.lineTo(0, 0)
  ctx.stroke()
  ctx.restore()
}

function Text(grid, uuid) {
  var el = document.createElement('div')
  el.style.display = 'flex'

  this.span = document.createElement('span')
  el.appendChild(this.span)

  this.text = 'Text'
  this.prefix = ''
  this.suffix = ''
  this.updateContent()

  Widget.call(this, grid, uuid, el)

  this.w = 3
  this.h = 1

  this.el.style.justifyContent = this.align = 'center'
  this.el.style.alignItems = this.valign = 'center'

  this.fontSize = 12
  this.span.style.fontSize = this.fontSize + 'pt'

  this.color = '#000000'
  this.background = ''
}

Widget.createSubclass(Text, {
  text: new Prop(String, undefined, function (val) {
    this.text = val
    this.updateContent()
  }),
  fontSize: new Prop(Number, undefined, function (val) {
    this.fontSize = val
    this.span.style.fontSize = val + 'pt'
  }),
  color: new Prop(String, undefined, function (val) {
    this.color = val
    this.span.style.color = val
  }).setIsColor(),
  background: new Prop(String, undefined, function (val) {
    this.background = val
    this.el.style.backgroundColor = val
  }).setIsColor(),
  align: new Prop(String, undefined, function (val) {
    this.align = val
    this.el.style.justifyContent = val
  }).setOptions(['flex-start', 'center', 'flex-end']),
  valign: new Prop(String, undefined, function (val) {
    this.valign = val
    this.el.style.alignItems = val
  }).setOptions(['flex-start', 'center', 'flex-end']),
  prefix: new Prop(String, undefined, function (val) {
    this.prefix = val
    this.updateContent()
  }),
  suffix: new Prop(String, undefined, function (val) {
    this.suffix = val
    this.updateContent()
  })
})

Widget.prototype.updateContent = function () {
  this.span.innerHTML = this.prefix + this.text + this.suffix
}

function Grid(manager, elementId, data) {
  this.manager = manager

  this.el = document.getElementById(elementId)
  this.widgets = []

  this.canvas = document.createElement('canvas')
  this.canvas.style.position = 'absolute'
  this.canvas.style.top = '0px'
  this.el.appendChild(this.canvas)

  this.tabs = []
  this.currentTabIdx = 0
  this.setTabCount(1)
  this.setCurrentTab(0)

  window.addEventListener('resize', this.onResize.bind(this))

  var focusedInput = null
  this.el.addEventListener(
    'touchstart',
    function (ev) {
      if (ev.target.tagName === 'INPUT') {
        ev.target.focus()
        focusedInput = ev.target
      } else if (focusedInput !== null) {
        focusedInput.blur()
        focusedInput = null
      }
    }.bind(this)
  )

  this.isSplit = 0
  this.offsetX = 0
  this.offsetY = 0
  this.scaleX = 1
  this.scaleY = 1

  this.reset(data)
}

Grid.prototype.setCurrentTab = function (idx) {
  this.tabs[this.currentTabIdx].style.display = 'none'
  this.tabs[idx].style.display = 'block'
  this.currentTabIdx = idx
  for (w of this.widgets) {
    if (w instanceof Switcher)
      w.value = idx
    w.applyState(w.getState())
  }
}

Grid.prototype.moveToTab = function (widget, tab, oldTab) {
  widget.tab = oldTab
  this.removeWidget(widget)
  widget.tab = tab
  this.addWidgetConstructed(widget)
}

Grid.prototype.setTabCount = function (count) {
  if (this.tabs.length === count || count <= 0) {
    return
  }

  if (this.tabs.length < count) {
    for (var i = this.tabs.length; i < count; ++i) {
      var t = document.createElement('div')
      t.style.width = '100%'
      t.style.height = '100%'
      t.style.display = 'none'
      this.el.appendChild(t) 
      this.tabs[i] = t
    }
  } else {
    while (this.tabs.length > count) {
      var t = this.tabs.pop()
      this.el.removeChild(t)
    }
    if (this.currentTabIdx >= this.tabs.length) {
      this.currentTabIdx = this.tabs.length - 1
    }
  }
}

Grid.prototype.reset = function (data) {
  this.COLS = data.cols
  this.ROWS = data.rows
  this.enableSplitting = !!data.enableSplitting
  this.drawGrid = !!data.drawGrid

  this.clear()

  for (var i = 0; i < data.widgets.length; ++i) {
    var w = data.widgets[i]
    this.addWidget(w.uuid, w.type, w['state'])
  }

  this.onResize()
}

Grid.prototype.onResize = function () {
  var w = this.el.clientWidth
  var h = this.el.clientHeight

  this.offsetX = 0
  this.offsetY = 0

  this.isSplit = this.shouldSplitGrid(w, h)
  if (!this.isSplit) {
    if (w > h) {
      this.offsetX = (w - h) / 2
      w = h
    }

    this.scaleX = w / this.COLS
    this.scaleY = h / this.ROWS

    this.canvas.style.width = '' + this.el.clientWidth + 'px'
    this.canvas.style.height = '' + this.el.clientHeight + 'px'
    this.canvas.width = this.el.clientWidth + 2
    this.canvas.height = this.el.clientHeight + 2

    this.drawGridDots(this.COLS, this.ROWS)
  } else {
    this.scaleX = w / (this.COLS * 2)
    this.scaleY = h / Math.round(this.ROWS / 2)

    this.canvas.style.width = '' + this.el.clientWidth + 'px'
    this.canvas.style.height = '' + this.el.clientHeight + 'px'
    this.canvas.width = this.el.clientWidth + 2
    this.canvas.height = this.el.clientHeight + 2

    this.drawGridDots(this.COLS * 2, this.ROWS / 2)
  }

  var len = this.widgets.length
  for (var i = 0; i < len; ++i) {
    this.widgets[i].updatePosition()
  }
}

Grid.prototype.calculatePxPos = function (w) {
  var res = {
    w: w.w * this.scaleX,
    h: w.h * this.scaleY
  }

  if (!this.isSplit) {
    res.x = this.offsetX + w.x * this.scaleX
    res.y = this.offsetY + w.y * this.scaleY
  } else {
    var centerY = Math.round(this.ROWS / 2)
    if (w.y >= centerY) {
      res.x = w.x + this.COLS
      res.y = w.y - centerY
    } else {
      res.x = w.x
      res.y = w.y
    }
    res.x *= this.scaleX
    res.y *= this.scaleY
  }

  return res
}

Grid.prototype.shouldSplitGrid = function (w, h) {
  if (this.enableSplitting !== true || w <= h) return false

  var centerY = this.ROWS / 2

  var len = this.widgets.length
  for (var i = 0; i < len; ++i) {
    var w = this.widgets[i]
    if (w.y < centerY && w.y + w.h > centerY) return false
  }
  return true
}

Grid.prototype.drawGridDots = function (cols, rows) {
  if (!this.drawGrid) return

  var ctx = this.canvas.getContext('2d')

  ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)
  ctx.fillStyle = '#777'

  for (var x = 0; x <= cols; ++x) {
    for (var y = 0; y <= rows; ++y) {
      ctx.beginPath()
      ctx.arc(
        this.offsetX + x * this.scaleX + 1,
        this.offsetY + y * this.scaleY + 1,
        1,
        0,
        Math.PI * 2,
        false
      )
      ctx.fill()
    }
  }
}

Grid.prototype.addWidget = function (uuid, typeName, extra) {
  try {
    var w = new window[typeName](this, uuid)
  } catch (e) {
    console.log(typeName, e)
    return
  }
  w.applyState(extra)
  this.addWidgetConstructed(w)
}

Grid.prototype.addWidgetConstructed = function (widget) {
  widget.updatePosition()
  widget.setEventListener(this.onWidgetEvent.bind(this))

  
  if (this.tabs.length <= widget.tab) 
    this.setTabCount(widget.tab + 1)
  this.tabs[widget.tab].appendChild(widget.el)
  this.widgets.push(widget)
}

Grid.prototype.removeWidget = function (widget) {
  var idx = this.widgets.indexOf(widget)
  if (idx === -1) return false

  this.tabs[widget.tab].removeChild(widget.el)
  this.widgets.splice(idx, 1)
  return true
}

Grid.prototype.clear = function () {
  var len = this.widgets.length
  for (var i = 0; i < len; ++i) {
    var w = this.widgets[i]
    this.tabs[w.tab].removeChild(w.el)
  }
  this.widgets = []
  this.setTabCount(1)
}

Grid.prototype.onWidgetEvent = function (w, name, extra, mustArrive, callback) {
  //console.log("Event from " + w.uuid + ": " + name + " " + JSON.stringify(extra));

  if (this.manager === null) return

  var data = {
    id: w.uuid,
    ev: name
  }

  if (extra !== undefined && extra !== null) {
    data['st'] = extra
  }

  if (mustArrive !== false) {
    this.manager.sendMustArrive('_gev', data, false, callback)
  } else {
    this.manager.send('_gev', data)
  }
}

Grid.prototype.update = function (diffMs) {
  var len = this.widgets.length
  for (var i = 0; i < len; ++i) {
    var w = this.widgets[i]
    w.update(diffMs)
  }
}

Grid.prototype.onMessageState = function (data) {
  var len = this.widgets.length
  for (var i = 0; i < len; ++i) {
    var w = this.widgets[i]
    var state = data[w.uuid.toString()]
    if (state !== undefined) {
      w.applyState(state)
    }
  }
}

Grid.prototype.getWidgetByUuid = function (uuid) {
  var len = this.widgets.length
  for (var i = 0; i < len; ++i) {
    var w = this.widgets[i]
    if (w.uuid === uuid) {
      return w
    }
  }
  return null
}

Grid.prototype.getWidgetAtPos = function (x, y) {
  var len = this.widgets.length
  for (var i = len - 1; i >= 0; --i) {
    var w = this.widgets[i]
    var r = w.el.getBoundingClientRect()
    if (r.left <= x && r.right >= x && r.top <= y && r.bottom >= y) {
      return w
    }
  }
  return null
}

Grid.prototype.roundToPrecision = function (x, precision) {
  var y = +x + (precision === undefined ? 0.5 : precision / 2)
  return y - (y % (precision === undefined ? 1 : +precision))
}

Grid.prototype.pxPosToCoordinates = function (x, y) {
  var rect = this.el.getBoundingClientRect()
  x -= rect.left
  y -= rect.top

  x = this.roundToPrecision(x / this.scaleX, 0.5)
  y = this.roundToPrecision(y / this.scaleY, 0.5)
  return {
    x: x,
    y: y
  }
}

Grid.prototype.tryMoveWidget = function (w, x, y) {
  var coords = this.pxPosToCoordinates(x, y)
  x = coords.x
  y = coords.y
  if (x == w.x && y == w.y) return false

  w.x = Math.min(this.COLS - 1, Math.max(-w.w + 1, x))
  w.y = Math.min(this.ROWS - 1, Math.max(-w.h + 1, y))
  w.updatePosition()
  return true
}

Grid.prototype.tryScaleWidget = function (widget, r, b) {
  var coords = this.pxPosToCoordinates(r, b)

  var w = coords.x - widget.x
  var h = coords.y - widget.y

  if (w == widget.w && h == widget.h) return false

  widget.w = Math.min(this.COLS, Math.max(0.5, w))
  widget.h = Math.min(this.ROWS, Math.max(0.5, h))
  widget.updatePosition()
  return true
}

window.addEventListener('load', function () {
  // Detect captive portal and show message
  var ua = window.navigator.userAgent;
  if(ua.indexOf("; wv)") !== -1 && ua.indexOf("Chrome/") !== -1 && ua.indexOf("Version/") !== -1 && ua.indexOf("Android") !== -1) {
    document.getElementById("main-ui").style.display = "none";
    document.getElementById("captive-msg").style.display = "block";

    document.getElementById("captive-url").innerText = window.location.origin;
    document.getElementById("captive-link").href = "intent://" + window.location.hostname + "#Intent;scheme=http;end";
    return;
  }

  var man = new Manager('log', 'grid')

  // Detect whether we're running in RBController or just web browser
  if("RbGravitySensor" in window || window.navigator.userAgent.toLowerCase().indexOf("rbcontroller") !== -1) {
    man.start('ws://localhost:9000', false);
  } else {
    man.start('ws://' + window.location.host, true)
  }
})

function closeCaptivePortalNotice() {
  document.getElementById('main-ui').style.display = "block";
  document.getElementById('captive-msg').style.display = 'none'
}

