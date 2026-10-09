(() => {
  "use strict";

  const model = window.GOGOGUARD_GUIDE_MODEL;
  if (!model || !model.system || !model.teaching || !model.diagnostics || !model.health) {
    document.querySelector("#model-state").textContent = "事实模型缺失";
    throw new Error("GoGoGuard guide model is unavailable");
  }

  const { system, teaching, diagnostics, health } = model;
  const $ = (selector, root = document) => root.querySelector(selector);
  const $$ = (selector, root = document) => [...root.querySelectorAll(selector)];
  const escapeHtml = (value) => String(value ?? "").replace(/[&<>'"]/g, (char) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;" }[char]));
  const glossary = new Map(teaching.glossary.map((item) => [item.term, item.plain]));
  const glossaryTerms = [...glossary.keys()].sort((a, b) => b.length - a.length);
  const termPattern = new RegExp(`(${glossaryTerms.map((term) => term.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")).join("|")})`, "g");
  const statusLabel = (status) => system.statuses[status]?.label || status || "❓ Unknown";
  const state = { localization: 0, depth: "L1", navigation: 0, scenario: "change", health: "findings", issue: null, check: 0, trail: [], conceptFrame: 0, trackingFrame: 0 };

  function terms(value) {
    const safe = escapeHtml(value);
    return safe.replace(termPattern, (term) => `<span class="term" tabindex="0" data-term="${escapeHtml(term)}">${term}</span>`);
  }

  function evidenceLinks(paths = []) {
    return paths.map((path) => `<a class="evidence-chip" href="../${encodeURI(path)}" target="_blank" rel="noreferrer">${escapeHtml(path)}</a>`).join("");
  }

  function showToast(message) {
    const toast = $("#toast");
    toast.textContent = message;
    toast.classList.add("show");
    clearTimeout(showToast.timer);
    showToast.timer = setTimeout(() => toast.classList.remove("show"), 1800);
  }

  function openModal(html) {
    $("#modal-content").innerHTML = html;
    $("#detail-modal").classList.add("open");
    $("#detail-modal").setAttribute("aria-hidden", "false");
    document.body.style.overflow = "hidden";
  }

  function closeModal() {
    $("#detail-modal").classList.remove("open");
    $("#detail-modal").setAttribute("aria-hidden", "true");
    document.body.style.overflow = "";
  }

  function setupShell() {
    $("#model-state").textContent = `${system.scope.sourceCommit.slice(0, 7)} · r${system.scope.latestFieldReceipt.replace(/^r/, "")}`;
    $("#menu-button").addEventListener("click", () => {
      const open = $("#side-rail").classList.toggle("open");
      $("#menu-button").setAttribute("aria-expanded", String(open));
    });
    $$(".side-rail a").forEach((link) => link.addEventListener("click", () => $("#side-rail").classList.remove("open")));
    $("#term-toggle").addEventListener("click", (event) => {
      const off = document.body.classList.toggle("terms-off");
      event.currentTarget.textContent = `术语解释：${off ? "关" : "开"}`;
      event.currentTarget.setAttribute("aria-pressed", String(!off));
    });
    $$('[data-close-modal]').forEach((node) => node.addEventListener("click", closeModal));
    document.addEventListener("keydown", (event) => { if (event.key === "Escape") closeModal(); });

    const tooltip = $("#term-tooltip");
    const showTerm = (target) => {
      if (!target.classList?.contains("term") || document.body.classList.contains("terms-off")) return;
      tooltip.innerHTML = `<b>${escapeHtml(target.dataset.term)}</b><br>${escapeHtml(glossary.get(target.dataset.term) || "")}`;
      const rect = target.getBoundingClientRect();
      tooltip.style.left = `${Math.min(window.innerWidth - 295, Math.max(10, rect.left))}px`;
      tooltip.style.top = `${Math.min(window.innerHeight - 110, rect.bottom + 8)}px`;
      tooltip.classList.add("show");
    };
    document.addEventListener("mouseover", (event) => showTerm(event.target));
    document.addEventListener("focusin", (event) => showTerm(event.target));
    document.addEventListener("mouseout", (event) => { if (event.target.classList?.contains("term")) tooltip.classList.remove("show"); });
    document.addEventListener("focusout", (event) => { if (event.target.classList?.contains("term")) tooltip.classList.remove("show"); });

    const sections = $$('[data-title]');
    const observer = new IntersectionObserver((entries) => {
      const visible = entries.filter((entry) => entry.isIntersecting).sort((a, b) => b.intersectionRatio - a.intersectionRatio)[0];
      if (!visible) return;
      $("#current-section").textContent = visible.target.dataset.title;
      $$('[data-nav]').forEach((link) => link.classList.toggle("active", link.dataset.nav === visible.target.id));
    }, { rootMargin: "-20% 0px -65% 0px", threshold: [0, .1, .3] });
    sections.forEach((section) => observer.observe(section));
    window.addEventListener("scroll", () => {
      const maximum = document.documentElement.scrollHeight - window.innerHeight;
      $("#reading-progress").style.width = `${maximum > 0 ? window.scrollY / maximum * 100 : 0}%`;
    }, { passive: true });
  }

  function renderIdentity() {
    const id = system.identity;
    $("#identity-row").innerHTML = [
      `当前地图 ${id.mapVersion}`,
      `路线 ${id.routeLengthM.toFixed(1)}m`,
      `${id.pcdPoints.toLocaleString()} 个历史点`,
      `field generation ${system.scope.fieldGeneration}`,
      `现场证据 ${system.scope.latestFieldReceipt}`
    ].map((item) => `<span>${escapeHtml(item)}</span>`).join("");
  }

  function renderBoundaries() {
    const board = $("#boundary-board");
    board.innerHTML = system.boundaries.map((item, index) => `<button type="button" class="boundary-card ${index === 0 ? "active" : ""}" data-boundary="${escapeHtml(item.id)}"><span>0${index + 1}</span><b>${escapeHtml(item.name)}</b><small>${escapeHtml(item.owns[0])}</small></button>`).join("");
    const select = (id) => {
      const item = system.boundaries.find((entry) => entry.id === id);
      $$(".boundary-card", board).forEach((card) => card.classList.toggle("active", card.dataset.boundary === id));
      $("#boundary-detail").innerHTML = `<div class="detail-grid"><div><span class="status-badge">${statusLabel(item.status)}</span><h3>${escapeHtml(item.name)}</h3></div><div><h4>现实职责</h4><ul>${item.owns.map((value) => `<li>${terms(value)}</li>`).join("")}</ul></div><div><h4>明确不负责</h4><ul>${item.doesNotOwn.map((value) => `<li>${terms(value)}</li>`).join("")}</ul><h4 style="margin-top:12px">证据</h4>${evidenceLinks(item.evidence)}</div></div>`;
    };
    board.addEventListener("click", (event) => { const card = event.target.closest("[data-boundary]"); if (card) select(card.dataset.boundary); });
    select(system.boundaries[0].id);
  }

  function renderMastery() {
    const track = $("#mastery-track");
    track.innerHTML = teaching.masteryChapters.map((chapter, index) => `<button type="button" class="chapter-button ${index === 0 ? "active" : ""}" data-chapter="${escapeHtml(chapter.id)}"><span>${escapeHtml(chapter.minutes)}</span><b>${escapeHtml(chapter.title)}</b><small>${escapeHtml(chapter.promise)}</small></button>`).join("");
    const select = (id) => {
      const chapter = teaching.masteryChapters.find((entry) => entry.id === id);
      $$(".chapter-button", track).forEach((button) => button.classList.toggle("active", button.dataset.chapter === id));
      $("#mastery-detail").innerHTML = `<div class="minutes">${escapeHtml(chapter.minutes)}</div><div><h3>${escapeHtml(chapter.title)}</h3><p>${terms(chapter.promise)} 这一段只引入完成理解所需的新词；需要工程细节时再进入 L3/L4。</p></div><a href="#${escapeHtml(chapter.anchor)}">进入这一段 →</a>`;
    };
    track.addEventListener("click", (event) => { const button = event.target.closest("[data-chapter]"); if (button) select(button.dataset.chapter); });
    select(teaching.masteryChapters[0].id);
  }

  function canvasContext(selector) {
    const canvas = $(selector);
    return { canvas, ctx: canvas.getContext("2d") };
  }

  function drawGrid(ctx, width, height, color = "rgba(255,255,255,.07)", size = 42) {
    ctx.strokeStyle = color; ctx.lineWidth = 1;
    for (let x = 0; x <= width; x += size) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke(); }
    for (let y = 0; y <= height; y += size) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }
  }

  function label(ctx, text, x, y, color = "rgba(255,255,255,.7)", size = 18, align = "left") {
    ctx.fillStyle = color; ctx.font = `600 ${size}px system-ui`; ctx.textAlign = align; ctx.fillText(text, x, y);
  }

  function drawDog(ctx, x, y, angle = 0, color = "#e76f3d") {
    ctx.save(); ctx.translate(x, y); ctx.rotate(angle); ctx.fillStyle = color;
    ctx.beginPath(); ctx.roundRect(-35, -20, 62, 40, 11); ctx.fill();
    ctx.beginPath(); ctx.roundRect(22, -15, 25, 27, 8); ctx.fill();
    ctx.strokeStyle = color; ctx.lineWidth = 7;
    [[-25,-13],[-5,-13],[-25,13],[-5,13]].forEach(([lx, ly]) => { ctx.beginPath();ctx.moveTo(lx,ly);ctx.lineTo(lx-2,ly + (ly < 0 ? -18 : 18));ctx.stroke(); });
    ctx.strokeStyle = "#f9e6d8";ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(40,0);ctx.stroke();
    ctx.restore();
  }

  function seededPoints(count, seed = 4) {
    let value = seed; const points = [];
    for (let i = 0; i < count; i += 1) { value = (value * 9301 + 49297) % 233280; const a = value / 233280; value = (value * 9301 + 49297) % 233280; const b = value / 233280; points.push([a, b]); }
    return points;
  }

  const pointSeed = seededPoints(190, 17);
  function drawPointStructure(ctx, ox, oy, rotation, color, alpha = 1, scale = 1) {
    ctx.save();ctx.translate(ox,oy);ctx.rotate(rotation);ctx.fillStyle=color;ctx.globalAlpha=alpha;
    const structure = [];
    for (let x=-180;x<=180;x+=12) structure.push([x,-95 + Math.sin(x/40)*4]);
    for (let y=-95;y<=100;y+=11) structure.push([-175,y]);
    for (let a=0;a<Math.PI*2;a+=.16) structure.push([72+Math.cos(a)*45,25+Math.sin(a)*45]);
    pointSeed.slice(0,50).forEach(([a,b])=>structure.push([-130+a*260,-65+b*150]));
    structure.forEach(([x,y])=>{ctx.beginPath();ctx.arc(x*scale,y*scale,2.2,0,Math.PI*2);ctx.fill();});ctx.restore();
  }

  function drawConcept(index, options = {}) {
    cancelAnimationFrame(state.conceptFrame);
    const { canvas, ctx } = canvasContext("#concept-canvas"); const w=canvas.width,h=canvas.height;
    ctx.clearRect(0,0,w,h);ctx.fillStyle="#142521";ctx.fillRect(0,0,w,h);drawGrid(ctx,w,h);
    if (index === 0) {
      ctx.strokeStyle="rgba(185,219,97,.6)";ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(160,390);ctx.lineTo(920,390);ctx.stroke();ctx.beginPath();ctx.moveTo(160,390);ctx.lineTo(160,100);ctx.stroke();
      label(ctx,"X",935,398,"#b9db61",18);label(ctx,"Y",150,82,"#b9db61",18);drawDog(ctx,560,270,-.3);ctx.strokeStyle="#e76f3d";ctx.lineWidth=4;ctx.beginPath();ctx.arc(560,270,95,-.3,.6);ctx.stroke();
      label(ctx,"位置：X / Y",220,455);label(ctx,"朝向：狗头方向",650,190,"#f3c5ae");label(ctx,"这组位置与朝向 → Pose",540,65,"white",24,"center");
    } else if (index === 1) {
      const tilt=(options.tilt ?? 18)*Math.PI/180;drawDog(ctx,540,270,tilt);const colors=["#e76f3d","#b9db61","#5fa8d3"];[[1,0,"加速度 X"],[0,-1,"加速度 Y"],[-.65,.7,"旋转变化"]].forEach(([dx,dy,t],i)=>{ctx.strokeStyle=colors[i];ctx.lineWidth=4;ctx.beginPath();ctx.moveTo(540,270);ctx.lineTo(540+dx*175,270+dy*135);ctx.stroke();label(ctx,t,540+dx*190,270+dy*150,colors[i],16,dx<0?"right":"left")});label(ctx,"它感到“刚刚怎样动了”",540,70,"white",26,"center");label(ctx,"短时间很敏感 · 单独累加会把误差也累加",540,455,"rgba(255,255,255,.55)",17,"center");
    } else if (index === 2) {
      const phase=options.phase??0;drawDog(ctx,540,290,0);ctx.fillStyle="#b9db61";ctx.beginPath();ctx.arc(552,272,7,0,Math.PI*2);ctx.fill();
      const walls=[{x:150,y:115,w:22,h:300},{x:860,y:95,w:25,h:330},{x:300,y:100,w:430,h:18},{x:340,y:395,w:380,h:17}];ctx.fillStyle="rgba(255,255,255,.16)";walls.forEach(r=>ctx.fillRect(r.x,r.y,r.w,r.h));
      for(let i=0;i<80;i++){const a=phase+i/80*Math.PI*2;let d=360;const dx=Math.cos(a),dy=Math.sin(a);if(Math.abs(dx)>.1){const tx=(dx>0?860:172)-552;const t=tx/dx;if(t>0)d=Math.min(d,t)}if(Math.abs(dy)>.1){const ty=(dy>0?395:118)-272;const t=ty/dy;if(t>0)d=Math.min(d,t)}const ex=552+dx*d,ey=272+dy*d;ctx.strokeStyle=`rgba(185,219,97,${i%8===0?.55:.08})`;ctx.lineWidth=i%8===0?2:1;ctx.beginPath();ctx.moveTo(552,272);ctx.lineTo(ex,ey);ctx.stroke();ctx.fillStyle="#e8f7b5";ctx.beginPath();ctx.arc(ex,ey,2.3,0,Math.PI*2);ctx.fill();}
      label(ctx,"发出激光 → 碰到表面 → 返回距离 → 空间点",540,58,"white",23,"center");label(ctx,"许多点一起勾出周围形状",540,472,"rgba(255,255,255,.6)",17,"center");
      if(options.animate!==false){const loop=()=>{if(state.localization!==2)return;drawConcept(2,{phase:(performance.now()/1600)%6.28,animate:false});state.conceptFrame=requestAnimationFrame(loop)};state.conceptFrame=requestAnimationFrame(loop)}
    } else if (index === 3 || index === 7) {
      const x=Number(options.x??68), y=Number(options.y??-38), r=Number(options.r??20)*Math.PI/180;drawPointStructure(ctx,510,270,0,"#b9db61",.65,1);drawPointStructure(ctx,510+x,270+y,r,"#5fa8d3",.85,1);label(ctx,index===3?"同一世界的两次观察":"历史 PCD 与当前短时扫描",540,50,"white",24,"center");label(ctx,"绿色：参照结构",110,465,"#b9db61",15);label(ctx,"蓝色：可移动的当前观察",800,465,"#5fa8d3",15);const score=Math.max(0,100-Math.abs(x)*.45-Math.abs(y)*.45-Math.abs(r*180/Math.PI)*1.1);label(ctx,`示意重合度 ${score.toFixed(0)}%`,540,485,score>78?"#b9db61":"#f3c5ae",18,"center");
    } else if (index === 4) {
      const error=Number(options.error??60);ctx.strokeStyle="#b9db61";ctx.lineWidth=4;ctx.beginPath();for(let x=100;x<=970;x+=10){const y=300+Math.sin(x/110)*45;if(x===100)ctx.moveTo(x,y);else ctx.lineTo(x,y)}ctx.stroke();ctx.strokeStyle="#e76f3d";ctx.setLineDash([10,8]);ctx.beginPath();for(let x=100;x<=970;x+=10){const k=(x-100)/870;const y=300+Math.sin(x/110)*45+k*error;if(x===100)ctx.moveTo(x,y);else ctx.lineTo(x,y)}ctx.stroke();ctx.setLineDash([]);label(ctx,"真实轨迹",130,250,"#b9db61",16);label(ctx,"只累加小误差后的估计",780,405,"#e76f3d",16);label(ctx,"每一步都只差一点，长时间后仍会明显分开",540,65,"white",23,"center");
    } else if (index === 5) {
      drawDog(ctx,540,290,0);ctx.strokeStyle="#5fa8d3";ctx.lineWidth=3;for(let i=0;i<4;i++){ctx.beginPath();ctx.moveTo(135,150+i*42);ctx.lineTo(440,265);ctx.stroke()}ctx.strokeStyle="#e76f3d";ctx.beginPath();ctx.moveTo(190,390);ctx.lineTo(440,310);ctx.stroke();label(ctx,"LiDAR：周围表面",110,120,"#5fa8d3",18);label(ctx,"IMU：快速运动线索",110,430,"#e76f3d",18);ctx.fillStyle="#b9db61";ctx.beginPath();ctx.roundRect(690,220,230,105,16);ctx.fill();label(ctx,"连续相对运动",805,267,"#183028",20,"center");label(ctx,"+ 去畸变扫描",805,295,"#183028",16,"center");label(ctx,"FAST-LIO 把两种互补信息反复结合",540,62,"white",23,"center");
    } else if (index === 6) {
      ctx.strokeStyle="#b9db61";ctx.lineWidth=3;ctx.strokeRect(100,100,880,330);ctx.strokeStyle="#e76f3d";ctx.setLineDash([9,8]);ctx.strokeRect(410,230,380,170);ctx.setLineDash([]);drawDog(ctx,610,315,-.15);label(ctx,"历史场地：跨启动保持固定",120,85,"#b9db61",18);label(ctx,"本次开机后的相对世界",425,218,"#e76f3d",16);label(ctx,"“从开机点走了 8m” ≠ “在场地地图的哪里”",540,480,"white",21,"center");
    } else if (index === 8) {
      ctx.strokeStyle="#b9db61";ctx.lineWidth=3;ctx.strokeRect(110,70,860,380);ctx.save();ctx.translate(520,255);ctx.rotate(.12);ctx.strokeStyle="#5fa8d3";ctx.strokeRect(-260,-125,520,250);ctx.restore();drawDog(ctx,570,275,.1);label(ctx,"map：固定在场地",135,105,"#b9db61",18);label(ctx,"odom：开机后的透明坐标纸",300,190,"#5fa8d3",18);ctx.strokeStyle="#e76f3d";ctx.lineWidth=5;ctx.beginPath();ctx.moveTo(780,120);ctx.lineTo(700,195);ctx.stroke();label(ctx,"VGICP 持续校正两张纸的关系",805,110,"#e76f3d",15,"center");
    } else {
      const nodes=[{x:145,t:"map",s:"历史场地"},{x:405,t:"odom",s:"连续相对"},{x:675,t:"base_link",s:"狗体基座"},{x:930,t:"LiDAR",s:"安装光心"}];nodes.forEach((n,i)=>{ctx.fillStyle=i===0?"#b9db61":i===1?"#5fa8d3":i===2?"#e76f3d":"#e1b83b";ctx.beginPath();ctx.arc(n.x,265,66,0,Math.PI*2);ctx.fill();label(ctx,n.t,n.x,260,"#17302a",19,"center");label(ctx,n.s,n.x,286,"rgba(23,48,42,.7)",13,"center");if(i<nodes.length-1){ctx.strokeStyle="rgba(255,255,255,.45)";ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(n.x+70,265);ctx.lineTo(nodes[i+1].x-70,265);ctx.stroke();}});label(ctx,"map→odom × odom→base_link = map→base_link Pose",540,75,"white",24,"center");label(ctx,"每一段只回答一种空间关系",540,465,"rgba(255,255,255,.55)",17,"center");
    }
  }

  function conceptControls(index) {
    const root=$("#concept-controls");root.innerHTML="";
    if(index===1){root.innerHTML='<label>模拟身体倾斜 <input id="tilt-control" type="range" min="-30" max="30" value="18"><output>18°</output></label>';$("#tilt-control").addEventListener("input",e=>{e.target.nextElementSibling.value=`${e.target.value}°`;drawConcept(index,{tilt:e.target.value})});}
    if(index===3||index===7){root.innerHTML='<label>平移 X <input id="align-x" type="range" min="-100" max="100" value="68"><output>68</output></label><label>平移 Y <input id="align-y" type="range" min="-100" max="100" value="-38"><output>-38</output></label><label>旋转 <input id="align-r" type="range" min="-40" max="40" value="20"><output>20°</output></label><button id="auto-align" type="button">自动对齐</button>';const redraw=()=>{const x=$("#align-x").value,y=$("#align-y").value,r=$("#align-r").value;$("#align-x").nextElementSibling.value=x;$("#align-y").nextElementSibling.value=y;$("#align-r").nextElementSibling.value=`${r}°`;drawConcept(index,{x,y,r})};$$('input',root).forEach(input=>input.addEventListener('input',redraw));$("#auto-align").addEventListener('click',()=>{let step=0;const from={x:+$("#align-x").value,y:+$("#align-y").value,r:+$("#align-r").value};const animate=()=>{step+=1;const p=Math.min(1,step/28),ease=1-Math.pow(1-p,3);$("#align-x").value=Math.round(from.x*(1-ease));$("#align-y").value=Math.round(from.y*(1-ease));$("#align-r").value=Math.round(from.r*(1-ease));redraw();if(p<1)requestAnimationFrame(animate)};animate()});}
    if(index===4){root.innerHTML='<label>累计误差 <input id="drift-control" type="range" min="0" max="115" value="60"><output>60</output></label>';$("#drift-control").addEventListener('input',e=>{e.target.nextElementSibling.value=e.target.value;drawConcept(index,{error:e.target.value})});}
  }

  function renderLocalizationDepth() {
    const lesson=teaching.localizationCourse[state.localization];let html="";
    if(state.depth==="L1") html=`<p>${terms(lesson.reality)} ${terms(lesson.success)}</p>`;
    if(state.depth==="L2") html=`<p><b>当前链路：</b>${terms(lesson.currentChain)}</p>`;
    if(state.depth==="L3") { const algorithm = state.localization===5?teaching.algorithmLessons.find(x=>x.id==="fastlio"):state.localization===7?teaching.algorithmLessons.find(x=>x.id==="vgicp"):null;html=algorithm?`<p><b>为什么能工作：</b>${terms(algorithm.whyWorks)}</p><ul>${algorithm.failures.slice(0,4).map(x=>`<li>${terms(x)}</li>`).join("")}</ul>`:`<p>${terms(lesson.intuition)} 先理解成功机制；失败条件只有在破坏这些条件时才成立。</p>`; }
    if(state.depth==="L4") html=`<p><b>当前实现：</b></p>${lesson.implementation.map(x=>`<span class="evidence-chip">${escapeHtml(x)}</span>`).join("")}<div style="margin-top:8px">${evidenceLinks(lesson.evidence)}</div>`;
    $("#localization-depth").innerHTML=html;
  }

  function renderLocalization() {
    const course=teaching.localizationCourse;const steps=$("#localization-steps");
    steps.innerHTML=course.map((lesson,index)=>`<button type="button" role="tab" class="step-button ${index===state.localization?"active":""}" data-step="${index}" aria-selected="${index===state.localization}"><span>${String(index+1).padStart(2,"0")}</span><b>${escapeHtml(lesson.title)}</b></button>`).join("");
    const show=(index)=>{state.localization=index;const lesson=course[index];$$('[data-step]',steps).forEach(button=>{const active=+button.dataset.step===index;button.classList.toggle('active',active);button.setAttribute('aria-selected',String(active))});$("#localization-progress").textContent=`${index+1} / ${course.length}`;$("#localization-progress-bar").style.width=`${(index+1)/course.length*100}%`;$("#localization-number").textContent=`STEP ${String(index+1).padStart(2,"0")}`;$("#localization-budget").innerHTML=lesson.newTerms.length?`本节只引入 ${lesson.newTerms.length} 个新词：${lesson.newTerms.map(escapeHtml).join(" · ")}`:"本节不引入新技术词";$("#localization-title").innerHTML=terms(lesson.title);$("#localization-reality").innerHTML=terms(lesson.reality);$("#localization-chain").innerHTML=terms(lesson.currentChain);$("#localization-intuition").innerHTML=terms(lesson.intuition);$("#localization-term").innerHTML=terms(lesson.termDefinition);$("#localization-success").innerHTML=terms(lesson.success);$("#localization-prev").disabled=index===0;$("#localization-next").disabled=index===course.length-1;conceptControls(index);drawConcept(index);renderLocalizationDepth();};
    steps.addEventListener('click',e=>{const button=e.target.closest('[data-step]');if(button)show(+button.dataset.step)});$("#localization-prev").addEventListener('click',()=>show(Math.max(0,state.localization-1)));$("#localization-next").addEventListener('click',()=>show(Math.min(course.length-1,state.localization+1)));$$('[data-depth]').forEach(button=>button.addEventListener('click',()=>{state.depth=button.dataset.depth;$$('[data-depth]').forEach(x=>x.classList.toggle('active',x===button));renderLocalizationDepth()}));show(0);
  }

  const scenarios={
    change:{label:"A · 垃圾桶消失",title:"局部变化，但稳定墙面仍在",explain:"垃圾桶对应的点消失，只损失一小部分重合；墙、路沿和建筑仍形成明显正确解，所以可能继续成功。",risk:"通常可定位 · 仍要通过 inlier / fitness 门限"},
    repeat:{label:"B · 连续相同围墙",title:"多个位置看起来一样",explain:"几段结构产生相近重合度，算法可能看到多个好候选。初始先验、歧义检查和连续性只能降低风险，不能消灭物理上的多解。",risk:"错配风险高 · 破坏‘正确解明显唯一’"},
    seed:{label:"C · 初始位置太远",title:"真实解不在搜索范围",explain:"当前只搜索 map-versioned 启动区附近。若狗放在 5m/60° 范围外，即使地图没有变化，优化也可能找不到正确盆地。",risk:"高概率失败 · 这是先验边界，不只是阈值问题"}
  };

  function drawScenario(kind) {
    const {canvas,ctx}=canvasContext('#scenario-canvas'),w=canvas.width,h=canvas.height;ctx.clearRect(0,0,w,h);ctx.fillStyle="#10201d";ctx.fillRect(0,0,w,h);drawGrid(ctx,w,h,"rgba(255,255,255,.05)",45);label(ctx,"历史地图",190,45,"#b9db61",18,"center");label(ctx,"今天的当前扫描",665,45,"#5fa8d3",18,"center");
    if(kind==='change'){drawPointStructure(ctx,210,290,0,"#b9db61",.75,.7);drawPointStructure(ctx,680,290,0,"#5fa8d3",.8,.7);ctx.fillStyle="#e76f3d";ctx.fillRect(265,300,45,55);ctx.strokeStyle="#e76f3d";ctx.lineWidth=3;ctx.strokeRect(735,300,45,55);label(ctx,"昨天有垃圾桶",285,390,"#e76f3d",14,"center");label(ctx,"今天这里为空",755,390,"#e76f3d",14,"center");}
    if(kind==='repeat'){for(let i=0;i<5;i++){ctx.strokeStyle="#b9db61";ctx.lineWidth=4;ctx.strokeRect(70+i*160,145,95,260);for(let y=160;y<400;y+=18){ctx.fillStyle=i===2?"#5fa8d3":"rgba(185,219,97,.55)";ctx.beginPath();ctx.arc(85+i*160,y,3,0,Math.PI*2);ctx.fill()}}label(ctx,"相同结构产生多个近似好位置",450,485,"#f3c5ae",18,"center");}
    if(kind==='seed'){drawPointStructure(ctx,220,300,0,"#b9db61",.7,.68);drawPointStructure(ctx,700,300,.45,"#5fa8d3",.75,.68);ctx.strokeStyle="#e76f3d";ctx.lineWidth=3;ctx.setLineDash([8,7]);ctx.beginPath();ctx.arc(220,300,120,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);label(ctx,"允许搜索区",220,455,"#e76f3d",15,"center");label(ctx,"真实观察落在外面",700,455,"#5fa8d3",15,"center");}
  }

  function renderScenarios(){const root=$("#scenario-buttons");root.innerHTML=Object.entries(scenarios).map(([id,item],index)=>`<button type="button" class="${index===0?'active':''}" data-scenario="${id}">${escapeHtml(item.label)}</button>`).join('');const select=(id)=>{state.scenario=id;$$('[data-scenario]',root).forEach(b=>b.classList.toggle('active',b.dataset.scenario===id));const item=scenarios[id];$("#scenario-explanation").innerHTML=`<div class="scenario-result"><b>${escapeHtml(item.title)}</b><p>${terms(item.explain)}</p><p><strong>${escapeHtml(item.risk)}</strong></p></div>`;drawScenario(id)};root.addEventListener('click',e=>{const b=e.target.closest('[data-scenario]');if(b)select(b.dataset.scenario)});select('change')}

  function renderNavigation(){const course=teaching.navigationCourse,root=$("#navigation-steps");root.innerHTML=course.map((lesson,index)=>`<button type="button" class="nav-step ${index===0?'active':''}" data-nav-step="${index}"><span>${String(index+1).padStart(2,'0')}</span><b>${escapeHtml(lesson.title)}</b></button>`).join('');const show=(index)=>{state.navigation=index;const lesson=course[index];$$('[data-nav-step]',root).forEach(b=>b.classList.toggle('active',+b.dataset.navStep===index));$("#navigation-lesson").innerHTML=`<span class="term-budget">${lesson.newTerms.length?`本节新词：${lesson.newTerms.map(escapeHtml).join(' · ')}`:'本节不引入新词'}</span><h3>${terms(lesson.title)}</h3><div class="nav-lesson-grid"><div><span>现实问题</span><p>${terms(lesson.reality)}</p></div><div><span>先建立直觉</span><p>${terms(lesson.intuition)}</p></div><div><span>当前链路</span><p>${terms(lesson.currentChain)}</p></div><div><span>代码证据</span><p>${evidenceLinks(lesson.evidence)}</p></div></div>`};root.addEventListener('click',e=>{const b=e.target.closest('[data-nav-step]');if(b)show(+b.dataset.navStep)});show(0);renderMotionChain();setupTracking()}

  function renderMotionChain(){const flow=system.flows.find(x=>x.id==='motion');$("#motion-chain").innerHTML=flow.steps.map((step,index)=>`<div class="motion-node"><div><b>${escapeHtml(step.owner)}</b><small>${terms(step.label)}<br>→ ${escapeHtml(step.output)}</small></div>${index<flow.steps.length-1?'<i></i>':''}</div>`).join('')}

  function setupTracking(){cancelAnimationFrame(state.trackingFrame);const disturbance=$("#tracking-disturbance"),response=$("#tracking-response");const updateOutputs=()=>{$$('output','.tracking-controls')[0].value=`${(+disturbance.value/100*1.4).toFixed(2)}m`;$$('output','.tracking-controls')[1].value=`${response.value}%`};const run=()=>{cancelAnimationFrame(state.trackingFrame);let frame=0;const animate=()=>{frame+=1;drawTracking(Math.min(1,frame/150),+disturbance.value/100*1.4,+response.value/100);if(frame<150)state.trackingFrame=requestAnimationFrame(animate)};animate()};[disturbance,response].forEach(input=>input.addEventListener('input',()=>{updateOutputs();run()}));$("#tracking-run").addEventListener('click',run);updateOutputs();run()}

  function drawTracking(progress,disturbance,response){const {canvas,ctx}=canvasContext('#tracking-canvas'),w=canvas.width,h=canvas.height;ctx.clearRect(0,0,w,h);ctx.fillStyle="#142521";ctx.fillRect(0,0,w,h);drawGrid(ctx,w,h);const pathY=x=>305+Math.sin((x-100)/175)*65;ctx.strokeStyle="#5fa8d3";ctx.lineWidth=6;ctx.beginPath();for(let x=80;x<=1030;x+=8){const y=pathY(x);if(x===80)ctx.moveTo(x,y);else ctx.lineTo(x,y)}ctx.stroke();ctx.fillStyle="rgba(231,111,61,.14)";ctx.fillRect(445,160,135,280);label(ctx,"扰动区",512,145,"#e76f3d",15,"center");const x=100+progress*900;const impulse=Math.exp(-Math.pow((progress-.47)/.12,2))*disturbance*95;const recovery=progress>.47?Math.exp(-(progress-.47)*response*8):1;const offset=impulse*recovery;const y=pathY(x)+offset;ctx.strokeStyle="#e76f3d";ctx.lineWidth=4;ctx.beginPath();for(let p=0;p<=progress;p+=.01){const tx=100+p*900;const imp=Math.exp(-Math.pow((p-.47)/.12,2))*disturbance*95;const rec=p>.47?Math.exp(-(p-.47)*response*8):1;const ty=pathY(tx)+imp*rec;if(p===0)ctx.moveTo(tx,ty);else ctx.lineTo(tx,ty)}ctx.stroke();drawDog(ctx,x,y,Math.atan2(pathY(x+5)-pathY(x),5),"#e76f3d");ctx.strokeStyle="#b9db61";ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x,pathY(x));ctx.stroke();label(ctx,`横向误差 ${Math.abs(y-pathY(x)).toFixed(1)} px`,x+12,(y+pathY(x))/2,"#b9db61",13);const vy=Math.max(-.2,Math.min(.2,-(y-pathY(x))/220*response)),wz=Math.max(-.4,Math.min(.4,-(y-pathY(x))/160*response));$("#command-readout").innerHTML=`<span>vx <b>0.60 m/s</b></span><span>vy <b>${vy.toFixed(2)} m/s</b></span><span>wz <b>${wz.toFixed(2)} rad/s</b></span><span>循环 <b>15 Hz</b></span>`}

  function renderMaps(){const grid=$("#map-grid");const render=(filter='all')=>{const items=system.mapTypes.filter(item=>filter==='all'||filter==='live'&&item.live||filter==='stored'&&!item.live&&!['route','path'].includes(item.id)||filter==='plan'&&['route','path'].includes(item.id));grid.innerHTML=items.map(item=>`<button type="button" class="map-card" data-map="${item.id}"><span class="map-type"><i>${item.live?'LIVE':'STORED'}</i><i>${statusLabel(item.status).split(' ')[0]}</i></span><h3>${escapeHtml(item.name)}</h3><p>${terms(item.storage)}</p><footer><span>${item.live?'实时变化':'长期资产'}</span><span>${item.users.length} 个消费者</span></footer></button>`).join('')};$$('[data-filter]').forEach(button=>button.addEventListener('click',()=>{$$('[data-filter]').forEach(x=>x.classList.toggle('active',x===button));render(button.dataset.filter)}));grid.addEventListener('click',e=>{const card=e.target.closest('[data-map]');if(!card)return;const item=system.mapTypes.find(x=>x.id===card.dataset.map);openModal(`<span class="status-badge">${statusLabel(item.status)}</span><h2 id="modal-title">${escapeHtml(item.name)}</h2><h4>它是什么 / 存在哪里</h4><p>${terms(item.storage)}</p><h4>谁创建</h4><p>${escapeHtml(item.creator)}</p><h4>是否实时变化</h4><p>${item.live?'是，任务运行中更新':'否，作为版本化资产或任务参考保存'}</p><h4>谁使用</h4><ul>${item.users.map(x=>`<li>${terms(x)}</li>`).join('')}</ul>${item.layers?`<h4>组成层</h4><p>${item.layers.map(terms).join(' · ')}</p>`:''}${item.note?`<h4>最容易混淆的点</h4><p>${terms(item.note)}</p>`:''}`)});render();renderFrames()}

  function renderFrames(){$("#frame-diagram").innerHTML=system.frames.map(frame=>`<div class="frame-ring"><span>${escapeHtml(frame.id)}</span><i></i></div>`).join('');$("#frame-list").innerHTML=system.frames.map(frame=>`<div class="frame-item"><b>${escapeHtml(frame.id)}</b><p>${terms(frame.meaning)} · authority: ${terms(frame.authority)} · ${frame.drifts?'会累计漂移':'自身定义不漂移'}</p></div>`).join('')}

  const algorithmQuestions=[
    ["为什么需要它？","plainProblem"],["没有它会怎样？","ifAbsent"],["现实输入是什么？","realityInputs"],["代码输入是什么？","codeInputs"],["大概经过哪些步骤？","processSteps"],["输出是什么？","output"],["输出在现实里意味着什么？","realityMeaning"],["为什么能工作？","whyWorks"],["依赖哪些条件？","assumptions"],["什么时候会失败？","failures"],["为什么当前产品选择它？","whySelected"],["可替代方案是什么？","alternatives"],["产品代价是什么？","productTradeoff"],["当前关键参数？","parameters"],["当前实现与证据？","codeEvidence"]
  ];
  function answerHtml(value,key){if(Array.isArray(value)){const tag=key==='processSteps'?'ol':'ul';return `<${tag}>${value.map(x=>`<li>${terms(x)}</li>`).join('')}</${tag}>`;}return `<p>${terms(value)}</p>`}
  function renderAlgorithms(){const tabs=$("#algorithm-tabs"),root=$("#algorithm-class");tabs.innerHTML=teaching.algorithmLessons.map((algo,index)=>`<button type="button" class="${index===0?'active':''}" data-algo="${algo.id}">${escapeHtml(algo.name)}</button>`).join('');const show=id=>{const algo=teaching.algorithmLessons.find(x=>x.id===id);$$('[data-algo]',tabs).forEach(b=>b.classList.toggle('active',b.dataset.algo===id));root.innerHTML=`<div class="algorithm-intro"><span class="algorithm-status">${escapeHtml(algo.runtimeStatus)}</span><h3>${escapeHtml(algo.name)}</h3><p>${terms(algo.plainProblem)}</p><div class="algorithm-position"><span>它在当前链路中的位置</span><b>${terms(algo.chainPosition)}</b></div></div><div class="algorithm-questions">${algorithmQuestions.map(([q,key],index)=>`<div class="algorithm-question ${index===0?'open':''}"><button type="button"><span>${String(index+1).padStart(2,'0')}</span>${escapeHtml(q)}<i>${index===0?'−':'+'}</i></button><div class="algorithm-answer">${answerHtml(algo[key],key)}</div></div>`).join('')}</div>`;$$('.algorithm-question button',root).forEach(button=>button.addEventListener('click',()=>{const card=button.parentElement;card.classList.toggle('open');$('i',button).textContent=card.classList.contains('open')?'−':'+'}))};tabs.addEventListener('click',e=>{const b=e.target.closest('[data-algo]');if(b)show(b.dataset.algo)});show(teaching.algorithmLessons[0].id)}

  function renderDebugger(){const list=$("#symptom-list");list.innerHTML=diagnostics.fieldDebugger.map(item=>`<button type="button" class="symptom-button" data-issue="${item.id}">${escapeHtml(item.symptom)}</button>`).join('');const select=id=>{state.issue=diagnostics.fieldDebugger.find(x=>x.id===id);state.check=0;state.trail=[];$$('[data-issue]',list).forEach(b=>b.classList.toggle('active',b.dataset.issue===id));renderDebugSession()};list.addEventListener('click',e=>{const b=e.target.closest('[data-issue]');if(b)select(b.dataset.issue)});}
  function renderDebugSession(){const root=$("#debug-session"),issue=state.issue;if(!issue)return;const check=issue.checks[state.check];root.innerHTML=`<div class="debug-head"><div><h3>${escapeHtml(issue.symptom)}</h3><p>${terms(issue.startAt)}</p></div><button type="button" id="debug-reset">重新开始</button></div><div class="debug-principle"><b>先抓住判断原则：</b>${terms(issue.firstPrinciple)}</div>${state.trail.length?`<div class="debug-trail">${state.trail.map(x=>`<span>${escapeHtml(x)}</span>`).join('')}</div>`:''}${check?`<div class="debug-check"><span>CHECK ${String(state.check+1).padStart(2,'0')} / ${issue.checks.length}</span><h4>${terms(check.question)}</h4><p>${terms(check.why)}</p><div class="debug-answers"><button type="button" data-answer="yes">是 / 证据支持</button><button type="button" data-answer="no">否 / 证据不支持</button></div><div class="debug-evidence"><span>先看：${escapeHtml(check.module)}</span><span>证据：${escapeHtml(check.evidence)}</span></div></div>`:`<div class="debug-finish"><h4>这轮反向路径已经走完</h4><p>可能类别：${issue.likelyClasses.map(terms).join(' · ')}。这不是自动诊断结论；它告诉你应从哪一个箭头开始收集证据，再把现场问题归为 Bug、参数、算法、架构或硬件。</p></div>`}`;$("#debug-reset").addEventListener('click',()=>{state.check=0;state.trail=[];renderDebugSession()});$$('[data-answer]',root).forEach(button=>button.addEventListener('click',()=>{const yes=button.dataset.answer==='yes';state.trail.push(`${state.check+1}. ${yes?'是':'否'} → ${yes?check.yes:check.no}`);state.check+=1;renderDebugSession()}));}

  function renderParameters(){const select=$("#parameter-module");const modules=[...new Set(diagnostics.parameters.map(x=>x.module))].sort();select.innerHTML='<option value="all">全部模块</option>'+modules.map(x=>`<option value="${escapeHtml(x)}">${escapeHtml(x)}</option>`).join('');const grid=$("#parameter-grid");const render=()=>{const query=$("#parameter-search").value.trim().toLowerCase(),module=select.value;const items=diagnostics.parameters.filter(item=>(module==='all'||item.module===module)&&(!query||JSON.stringify(item).toLowerCase().includes(query)));grid.innerHTML=items.map(item=>`<article class="parameter-card" data-parameter="${item.id}" tabindex="0"><span>${escapeHtml(item.module)}</span><h3>${escapeHtml(item.name)}</h3><p>${terms(item.controls)}</p><div class="parameter-value"><span>当前有效值</span><b>${escapeHtml(item.current)}</b></div><div class="parameter-arrows"><span>↑ 增大：${escapeHtml(item.increase)}</span><span>↓ 减小：${escapeHtml(item.decrease)}</span></div></article>`).join('')||'<p>没有匹配参数。</p>'};const open=id=>{const item=diagnostics.parameters.find(x=>x.id===id);openModal(`<span class="status-badge">${statusLabel(item.status)}</span><h2 id="modal-title">${escapeHtml(item.name)}</h2><h4>它控制什么</h4><p>${terms(item.controls)}</p><h4>当前有效值 / 默认值</h4><p><b>${escapeHtml(item.current)}</b><br>${escapeHtml(item.default)}</p><h4>增大 / 减小</h4><p>↑ ${terms(item.increase)}<br>↓ ${terms(item.decrease)}</p><h4>可能改善</h4><p>${terms(item.improves)}</p><h4>副作用</h4><p>${terms(item.sideEffects)}</p><h4>为什么是当前值</h4><p>${terms(item.rationale)}</p><h4>来源与证据</h4><p>${escapeHtml(item.provenance)}</p>${evidenceLinks(item.evidence)}`)};$("#parameter-search").addEventListener('input',render);select.addEventListener('change',render);grid.addEventListener('click',e=>{const card=e.target.closest('[data-parameter]');if(card)open(card.dataset.parameter)});grid.addEventListener('keydown',e=>{const card=e.target.closest('[data-parameter]');if(card&&(e.key==='Enter'||e.key===' ')){e.preventDefault();open(card.dataset.parameter)}});render()}

  function renderHealth(){const snapshot=health.snapshot;$("#repo-snapshot").innerHTML=[['Commit',snapshot.commit.slice(0,7)],['跟踪文件',snapshot.trackedFiles],['ROS Packages',snapshot.rosPackagesInWorkspace],['Git Tags',snapshot.tags],['Branch upstream','未配置']].map(([labelText,value])=>`<div><b>${escapeHtml(value)}</b><span>${escapeHtml(labelText)}</span></div>`).join('');const show=kind=>{state.health=kind;$$('[data-health]').forEach(b=>b.classList.toggle('active',b.dataset.health===kind));const root=$("#health-content");if(kind==='findings')root.innerHTML=`<div class="finding-grid">${health.healthFindings.map(item=>`<article class="finding-card"><header><span>SEV-${item.severity} · ${statusLabel(item.status)}</span><small>${escapeHtml(item.category)}</small></header><h3>${escapeHtml(item.title)}</h3><p>${terms(item.whyItMatters)}</p><footer>下一步验证：${terms(item.recommendedNext)}</footer></article>`).join('')}</div>`;if(kind==='legacy'){root.innerHTML=`<div class="legacy-list">${health.legacyDecisions.map(item=>`<article class="legacy-card"><button type="button"><span>${statusLabel(item.status)}</span><b>${escapeHtml(item.name)}</b><small>展开删除判断 +</small></button><div class="legacy-detail"><div class="legacy-facts"><div><span>原来解决什么</span><p>${terms(item.originalProblem)}</p></div><div><span>当前替代者</span><p>${terms(item.currentReplacement)}</p></div><div><span>现在是否启动</span><p>${terms(item.runtimeStarted)}</p></div><div><span>Git 能证明什么</span><p>${terms(item.gitHistory)}</p></div><div><span>为什么判定非主链</span><p>${terms(item.whyInactive)}</p></div><div><span>删除风险</span><p>${terms(item.deletionRisk)}</p></div></div><h4>删除前验证</h4><ol class="validation-list">${item.validationBeforeDelete.map(x=>`<li>${terms(x)}</li>`).join('')}</ol><div>${evidenceLinks(item.codeRefs)}</div></div></article>`).join('')}</div>`;$$('.legacy-card>button',root).forEach(button=>button.addEventListener('click',()=>button.parentElement.classList.toggle('open')))}if(kind==='rules')root.innerHTML=`<div class="rule-grid">${health.governanceRules.map(rule=>`<article class="rule-card"><b>${escapeHtml(rule.owner)}</b><p>${terms(rule.rule)}</p></article>`).join('')}</div>`};$$('[data-health]').forEach(button=>button.addEventListener('click',()=>show(button.dataset.health)));show('findings')}

  function evidenceRecords(){const records=[];system.boundaries.forEach(x=>records.push({kind:'系统边界',title:x.name,summary:x.owns.join('；'),status:x.status,paths:x.evidence||[]}));system.modules.forEach(x=>records.push({kind:'运行模块',title:x.name,summary:`输入 ${x.inputs.join(', ')}；输出 ${x.outputs.join(', ')}`,status:x.status,paths:x.code||[]}));system.flows.forEach(x=>records.push({kind:'真实链路',title:x.name,summary:x.steps.map(s=>`${s.owner}→${s.output}`).join('；'),status:x.status,paths:x.evidence||[]}));teaching.algorithmLessons.forEach(x=>records.push({kind:'核心算法',title:x.name,summary:`${x.plainProblem} ${x.runtimeStatus}`,status:x.status,paths:x.codeEvidence||[]}));health.legacyDecisions.forEach(x=>records.push({kind:'Legacy 审计',title:x.name,summary:`${x.runtimeStarted} ${x.currentReplacement}`,status:x.status,paths:x.codeRefs||[]}));diagnostics.parameters.forEach(x=>records.push({kind:'参数',title:x.name,summary:`${x.current}；${x.controls}`,status:x.status,paths:x.evidence||[]}));system.externalDependencies.forEach(x=>records.push({kind:'外部依赖',title:x.name,summary:`GLIM ${x.glimCommit}; config ${x.configFileSetHash}`,status:x.status,paths:x.evidence||[]}));return records}
  function renderEvidence(){const records=evidenceRecords(),root=$("#evidence-results"),input=$("#evidence-search");const render=()=>{const q=input.value.trim().toLowerCase();const result=records.filter(record=>!q||JSON.stringify(record).toLowerCase().includes(q));$("#evidence-count").textContent=`${result.length} / ${records.length} 条`;root.innerHTML=result.slice(0,30).map(record=>`<article class="evidence-result"><header><b>${escapeHtml(record.title)}</b><span>${escapeHtml(record.kind)} · ${statusLabel(record.status)}</span></header><p>${terms(record.summary)}</p><div class="evidence-paths">${evidenceLinks(record.paths)}</div></article>`).join('')};input.addEventListener('input',render);render();renderExternal()}

  function renderExternal(){const x=system.externalDependencies[0];$("#external-card").innerHTML=`<span class="status-badge">${statusLabel(x.status)}</span><h3>GLIM：现在已知到什么程度</h3><p>第一版只能确认“外部云 pipeline”。第二版从现存 session 快照补到了精确 commit、有效配置哈希、加载插件、输入 Topic、一次实际运行质量和输出产物。仍不可伪称已拥有外部源码树或 worker 镜像身份。</p><div class="external-grid"><div><span>GLIM commit</span><b>${escapeHtml(x.glimCommit)}</b></div><div><span>glim_ros2 commit</span><b>${escapeHtml(x.glimRos2Commit)}</b></div><div><span>config file-set hash</span><b>${escapeHtml(x.configFileSetHash)}</b></div><div><span>有效插件</span><b>${x.effectivePlugins.map(escapeHtml).join('<br>')}</b></div><div><span>实际输入</span><b>${x.inputs.map(escapeHtml).join('<br>')}</b></div><div><span>实际产物</span><b>${x.outputs.map(escapeHtml).join('<br>')}</b></div></div><p><b>证据边界：</b>${escapeHtml(x.sourceAvailability)}</p><div>${evidenceLinks(x.evidence)}</div>`}

  function boot(){setupShell();renderIdentity();renderBoundaries();renderMastery();renderLocalization();renderScenarios();renderNavigation();renderMaps();renderAlgorithms();renderDebugger();renderParameters();renderHealth();renderEvidence();showToast('第二版事实模型已加载');}
  boot();
})();
