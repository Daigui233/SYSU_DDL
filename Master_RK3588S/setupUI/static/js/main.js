(function () {
    const maxStep = 5;
    let currentStep = 1;
    let cachedConfig = null;
    let wasdTimer = null;
    const pressedKeys = new Set();
    const wasdPose = { x: 0, y: 0.16, z: 0, yaw: 0 };

    function byId(id) {
        return document.getElementById(id);
    }

    function all(selector) {
        return Array.from(document.querySelectorAll(selector));
    }

    function setMessage(text, level) {
        const el = byId("status-message");
        if (!el) {
            return;
        }
        el.textContent = text || "";
        el.style.color = level === "bad" ? "#ff7474" : level === "ok" ? "#55ff99" : "";
    }

    function showStep(step) {
        currentStep = Math.max(1, Math.min(maxStep, step));
        all(".step-page").forEach((page) => {
            page.classList.toggle("active", page.id === `step-${currentStep}`);
        });

        const showControls = currentStep > 1;
        const controls = byId("wizard-controls");
        const progress = byId("progress-container");
        if (controls) {
            controls.style.display = showControls ? "flex" : "none";
        }
        if (progress) {
            progress.style.display = showControls ? "block" : "none";
        }

        const bar = byId("progress-bar");
        if (bar) {
            bar.style.width = `${Math.round(((currentStep - 1) / (maxStep - 1)) * 100)}%`;
        }
        all(".step-dot").forEach((dot) => {
            const dotStep = Number(dot.dataset.step || 0);
            dot.classList.toggle("active", dotStep <= Math.min(currentStep, 4));
        });

        const prev = byId("btn-prev");
        const next = byId("btn-next");
        const saveNext = byId("btn-save-next");
        const skip = byId("btn-skip");
        const reset = byId("btn-reset");
        if (prev) {
            prev.disabled = currentStep <= 2;
        }
        if (next) {
            next.style.display = currentStep === 3 || currentStep >= 5 ? "none" : "";
            next.textContent = currentStep === 4 ? "进入监控" : "下一步";
        }
        if (saveNext) {
            saveNext.style.display = currentStep === 3 ? "" : "none";
        }
        if (skip) {
            skip.style.display = currentStep === 3 ? "" : "none";
        }
        if (reset) {
            reset.style.display = currentStep === 5 ? "" : "none";
        }

        if (currentStep === 3) {
            loadConfig();
        }
        if (currentStep === 4) {
            runSelfCheck();
        }
    }

    async function fetchJson(url, options) {
        const response = await fetch(url, {
            cache: "no-store",
            headers: { "Content-Type": "application/json" },
            ...(options || {}),
        });
        const data = await response.json().catch(() => ({}));
        if (!response.ok) {
            throw new Error(data.error || `HTTP ${response.status}`);
        }
        return data;
    }

    function setValue(id, value) {
        const el = byId(id);
        if (el && value !== undefined && value !== null) {
            el.value = value;
        }
    }

    function getNumber(id, fallback) {
        const el = byId(id);
        const value = Number(el ? el.value : NaN);
        return Number.isFinite(value) ? value : fallback;
    }

    async function loadConfig() {
        if (cachedConfig) {
            fillConfig(cachedConfig);
            return cachedConfig;
        }
        try {
            cachedConfig = await fetchJson("/api/config");
            fillConfig(cachedConfig);
            setMessage("配置已从 dist/main_config.json 载入。", "ok");
        } catch (err) {
            setMessage(`配置读取失败：${err.message}`, "bad");
        }
        return cachedConfig;
    }

    function fillConfig(config) {
        const network = config && (config.network || config);
        if (!network) {
            return;
        }
        setValue("udp_target_ip", network.udp_target_ip || "127.0.0.1");
        setValue("udp_target_port", network.udp_target_port || network.control_port || 9006);
        setValue("unity_target_ip", network.unity_target_ip || "127.0.0.1");
        setValue("unity_sync_port", network.unity_sync_port || 9003);
        const pos = Array.isArray(network.pos_offset) ? network.pos_offset : [0, 0.16, 0];
        const euler = Array.isArray(network.euler_offset) ? network.euler_offset : [0, 0, 0];
        setValue("pos_offset_x", pos[0]);
        setValue("pos_offset_y", pos[1]);
        setValue("pos_offset_z", pos[2]);
        setValue("euler_offset_x", euler[0]);
        setValue("euler_offset_y", euler[1]);
        setValue("euler_offset_z", euler[2]);
        const wasdPort = byId("wasd-udp-port");
        if (wasdPort) {
            wasdPort.textContent = "9005";
        }
    }

    async function saveConfig() {
        const base = cachedConfig || (await loadConfig()) || {};
        const nextConfig = JSON.parse(JSON.stringify(base));
        const network = nextConfig.network || {};
        network.udp_target_ip = byId("udp_target_ip")?.value.trim() || "127.0.0.1";
        network.udp_target_port = getNumber("udp_target_port", 9006);
        network.unity_target_ip = byId("unity_target_ip")?.value.trim() || "127.0.0.1";
        network.unity_sync_port = getNumber("unity_sync_port", 9003);
        network.pos_offset = [
            getNumber("pos_offset_x", 0),
            getNumber("pos_offset_y", 0.16),
            getNumber("pos_offset_z", 0),
        ];
        network.euler_offset = [
            getNumber("euler_offset_x", 0),
            getNumber("euler_offset_y", 0),
            getNumber("euler_offset_z", 0),
        ];
        nextConfig.network = network;

        await fetchJson("/api/config", {
            method: "POST",
            body: JSON.stringify(nextConfig),
        });
        cachedConfig = nextConfig;
        setMessage("配置已保存。", "ok");
        showStep(4);
    }

    function setCheck(id, state, text, tip) {
        const item = byId(`check-${id}`);
        const status = byId(`status-${id}`);
        const fixTip = byId(`fix-${id}`);
        if (item) {
            item.style.borderColor = state === "ok" ? "rgba(70,242,143,.4)" : state === "bad" ? "rgba(255,90,99,.42)" : "";
        }
        if (status) {
            status.className = `check-status ${state === "ok" ? "status-ok" : state === "bad" ? "status-bad" : "status-loading"}`;
            status.textContent = text;
        }
        if (fixTip) {
            fixTip.textContent = tip || "";
        }
    }

    async function runSelfCheck() {
        setCheck("camera-connect", "loading", "检查中...", "");
        setCheck("camera-perm", "loading", "检查中...", "");
        setCheck("video-stream", "loading", "检查中...", "");
        try {
            const status = await fetchJson("/pose_status");
            setCheck("camera-connect", "ok", "WebUI 可达", `HTTP 端口 ${status.http_port || 9105}`);
            setCheck("camera-perm", status.packet_json ? "ok" : "bad", status.packet_json ? "配置可读" : "缺少状态文件", status.packet_json || "");
            const hasPose = Number(status.packet_count || 0) > 0;
            setCheck(
                "video-stream",
                hasPose ? "ok" : "loading",
                hasPose ? "已收到定位" : "等待定位",
                hasPose ? `AR 转发 ${status.udp_send_count || 0}` : "Windows 定位端应发送到 RK3588S:9005"
            );
            const summary = byId("summary-card");
            if (summary) {
                summary.style.display = "";
            }
            setMessage("自检完成。", "ok");
        } catch (err) {
            setCheck("camera-connect", "bad", "9105 不可达", err.message);
            setCheck("camera-perm", "loading", "未检查", "");
            setCheck("video-stream", "loading", "未检查", "");
            setMessage(`自检失败：${err.message}`, "bad");
        }
    }

    function wireUpload(boxId, inputId, progressId, barId, textId) {
        const box = byId(boxId);
        const input = byId(inputId);
        if (!box || !input) {
            return;
        }
        box.addEventListener("click", () => input.click());
        input.addEventListener("change", () => {
            const file = input.files && input.files[0];
            if (!file) {
                return;
            }
            const progress = byId(progressId);
            const bar = byId(barId);
            const text = byId(textId);
            if (progress) {
                progress.style.display = "block";
            }
            let pct = 0;
            const timer = window.setInterval(() => {
                pct = Math.min(100, pct + 25);
                if (bar) {
                    bar.style.width = `${pct}%`;
                }
                if (text) {
                    text.textContent = pct >= 100 ? `${file.name} 已选择` : `${pct}%`;
                }
                if (pct >= 100) {
                    window.clearInterval(timer);
                }
            }, 90);
        });
    }

    function copyStreamAddress() {
        const text = byId("stream-address")?.textContent.trim() || "";
        if (!text) {
            return;
        }
        if (navigator.clipboard) {
            navigator.clipboard.writeText(text).then(
                () => setMessage("融合流地址已复制。", "ok"),
                () => setMessage(text)
            );
        } else {
            setMessage(text);
        }
    }

    function updateWasdText() {
        const el = byId("wasd-status-text");
        if (!el) {
            return;
        }
        el.textContent = `X: ${wasdPose.x.toFixed(2)}   Y: ${wasdPose.y.toFixed(2)}   Z: ${wasdPose.z.toFixed(2)}   YAW: ${wasdPose.yaw.toFixed(1)}deg`;
    }

    async function sendWasdPose() {
        try {
            await fetchJson("/api/manual_pose", {
                method: "POST",
                body: JSON.stringify(wasdPose),
            });
        } catch (err) {
            setMessage(`WASD 投递失败：${err.message}`, "bad");
        }
    }

    function tickWasd() {
        const manualY = Number(byId("wasd-manual-y")?.value);
        if (Number.isFinite(manualY)) {
            wasdPose.y = manualY;
        }
        const yawRad = (wasdPose.yaw * Math.PI) / 180;
        const speed = 0.035;
        if (pressedKeys.has("a")) {
            wasdPose.yaw += 3;
        }
        if (pressedKeys.has("d")) {
            wasdPose.yaw -= 3;
        }
        if (pressedKeys.has("w")) {
            wasdPose.x += Math.cos(yawRad) * speed;
            wasdPose.z += Math.sin(yawRad) * speed;
        }
        if (pressedKeys.has("s")) {
            wasdPose.x -= Math.cos(yawRad) * speed;
            wasdPose.z -= Math.sin(yawRad) * speed;
        }
        wasdPose.x = Math.max(-5, Math.min(5, wasdPose.x));
        wasdPose.z = Math.max(-5, Math.min(5, wasdPose.z));
        wasdPose.yaw = ((wasdPose.yaw % 360) + 360) % 360;
        updateWasdText();
        if (pressedKeys.size > 0) {
            sendWasdPose();
        }
    }

    function setWasdActive(active) {
        const hud = byId("wasd-hud");
        const button = byId("btn-toggle-wasd");
        if (hud) {
            hud.style.display = active ? "block" : "none";
        }
        if (button) {
            button.textContent = active ? "⏸ 停止动态虚拟摇杆" : "▶ 激活动态虚拟摇杆";
            button.style.background = active ? "#56616a" : "#28a745";
            button.style.borderColor = active ? "#6c757d" : "#28a745";
        }
        if (active && !wasdTimer) {
            wasdTimer = window.setInterval(tickWasd, 90);
            setMessage("WASD 已激活，姿态将通过本机 9005 投递。", "ok");
        }
        if (!active && wasdTimer) {
            window.clearInterval(wasdTimer);
            wasdTimer = null;
            pressedKeys.clear();
            setMessage("WASD 已停止。");
        }
    }

    function downloadJsonExample() {
        const data = {
            type: "robot_position",
            pos: [1.0, 0.16, 1.5],
            euler: [0.0, 90.0, 0.0],
        };
        const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
        const url = URL.createObjectURL(blob);
        const link = document.createElement("a");
        link.href = url;
        link.download = "robot_position_example.json";
        link.click();
        URL.revokeObjectURL(url);
    }

    function refreshRecords() {
        const list = byId("match-records-list");
        if (list) {
            list.innerHTML = '<div style="text-align:center;color:var(--text-muted);padding:20px;">暂无本地裁判事件记录</div>';
        }
        setMessage("裁判事件列表已刷新。");
    }

    function bindEvents() {
        byId("btn-start-wizard")?.addEventListener("click", () => showStep(2));
        byId("btn-next")?.addEventListener("click", () => showStep(currentStep + 1));
        byId("btn-prev")?.addEventListener("click", () => showStep(currentStep - 1));
        byId("btn-skip")?.addEventListener("click", () => showStep(4));
        byId("btn-save-next")?.addEventListener("click", () => {
            saveConfig().catch((err) => setMessage(`保存失败：${err.message}`, "bad"));
        });
        byId("btn-reset")?.addEventListener("click", () => showStep(1));
        byId("btn-offset-default")?.addEventListener("click", () => {
            setValue("pos_offset_x", 0);
            setValue("pos_offset_y", 0.16);
            setValue("pos_offset_z", 0);
            setValue("euler_offset_x", 0);
            setValue("euler_offset_y", 0);
            setValue("euler_offset_z", 0);
            setMessage("相机偏移已填入默认值。", "ok");
        });
        byId("btn-recheck")?.addEventListener("click", runSelfCheck);
        byId("btn-stop-system")?.addEventListener("click", () => setMessage("请在终端停止 ar_receiver.py 进程。"));
        byId("btn-import-preset-a")?.addEventListener("click", () => setMessage("标准赛道 A 已在 dist 中可用。", "ok"));
        byId("btn-copy-stream")?.addEventListener("click", copyStreamAddress);
        byId("btn-start-preview")?.addEventListener("click", () => setMessage("ar_receiver.py 正在提供状态服务；OpenCV 预览窗口由后端进程控制。", "ok"));
        byId("btn-stop-preview")?.addEventListener("click", () => setMessage("预览窗口需要在后端进程内关闭。"));
        byId("btn-toggle-wasd")?.addEventListener("click", () => setWasdActive(!wasdTimer));
        byId("btn-download-json")?.addEventListener("click", downloadJsonExample);
        byId("btn-refresh-records")?.addEventListener("click", refreshRecords);
        byId("btn-close-viewer")?.addEventListener("click", () => {
            const modal = byId("referee-viewer-modal");
            if (modal) {
                modal.style.display = "none";
            }
        });

        wireUpload("scene-upload-box", "scene-file-input", "upload-progress-container", "upload-progress-bar", "upload-status-text");
        wireUpload("engine-upload-box", "engine-file-input", "engine-upload-progress", "engine-progress-bar", "engine-upload-status");

        document.addEventListener("keydown", (event) => {
            const key = event.key.toLowerCase();
            if (!wasdTimer || !["w", "a", "s", "d", "q"].includes(key)) {
                return;
            }
            event.preventDefault();
            if (key === "q") {
                setWasdActive(false);
                return;
            }
            pressedKeys.add(key);
        });

        document.addEventListener("keyup", (event) => {
            pressedKeys.delete(event.key.toLowerCase());
        });
    }

    window.addEventListener("DOMContentLoaded", () => {
        bindEvents();
        showStep(1);
        updateWasdText();
        refreshRecords();
    });
})();
