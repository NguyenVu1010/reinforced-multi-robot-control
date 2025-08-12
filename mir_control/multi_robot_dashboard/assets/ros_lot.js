/**
 * Chờ cho cấu trúc HTML của trang được tải hoàn toàn trước khi chạy bất kỳ mã nào.
 * Đây là cách chuẩn để tránh lỗi "race condition" khi script chạy trước khi phần tử tồn tại.
 */
document.addEventListener('DOMContentLoaded', () => {

    // --- Lấy các phần tử HTML mà Dash đã tạo ---
    // Bây giờ, dòng này sẽ chạy sau khi canvas chắc chắn đã tồn tại.
    const canvas = document.getElementById('map-canvas'); 
    const statusElement = document.getElementById('ros-status');
    const errorElement = document.getElementById('error-message');

    // Kiểm tra an toàn một lần nữa
    if (!canvas) {
        console.error("Lỗi nghiêm trọng: DOM đã tải nhưng vẫn không tìm thấy canvas#map-canvas.");
        return;
    }

    const ctx = canvas.getContext('2d');
    let pathLibrary = null;

    // --- Các hàm tiện ích để tìm và vẽ đường đi (không thay đổi) ---

    function findEdge(fromNode, toNode) {
        if (!pathLibrary) return null;
        for (const edge of pathLibrary.edges) {
            if (edge.from === fromNode && edge.to === toNode) return edge.path_points;
            if (edge.from === toNode && edge.to === fromNode) return [...edge.path_points].reverse();
        }
        return null;
    }

    function findTotalPath(routeNodes) {
        const totalPathPoints = [];
        errorElement.textContent = '';
        for (let i = 0; i < routeNodes.length - 1; i++) {
            const segmentPoints = findEdge(routeNodes[i], routeNodes[i + 1]);
            if (segmentPoints) {
                totalPathPoints.push(...(i === 0 ? segmentPoints : segmentPoints.slice(1)));
            } else {
                errorElement.textContent = `Lỗi: Không tìm thấy đường đi từ node ${routeNodes[i]} đến ${routeNodes[i + 1]}.`;
                return null;
            }
        }
        return totalPathPoints;
    }

    function drawPath(points) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        if (!points || points.length < 2) return;
        ctx.strokeStyle = '#007bff'; ctx.lineWidth = 4; ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.moveTo(points[0][0], points[0][1]);
        for (let i = 1; i < points.length; i++) ctx.lineTo(points[i][0], points[i][1]);
        ctx.stroke();
        ctx.beginPath(); ctx.fillStyle = 'green'; ctx.arc(points[0][0], points[0][1], 8, 0, 2 * Math.PI); ctx.fill();
        ctx.beginPath(); ctx.fillStyle = 'red'; ctx.arc(points[points.length - 1][0], points[points.length - 1][1], 8, 0, 2 * Math.PI); ctx.fill();
    }

    // --- Hàm chính để khởi tạo mọi thứ (không thay đổi) ---

    async function initialize() {
        statusElement.textContent = 'Đang khởi tạo...';
        try {
            const response = await fetch('/assets/path_library.json');
            if (!response.ok) throw new Error(`Không thể tải file (HTTP ${response.status})`);
            pathLibrary = await response.json();
            console.log("Tải thư viện đường đi thành công!");
        } catch (error) {
            errorElement.textContent = `Lỗi nghiêm trọng khi tải path_library.json: ${error.message}`;
            return;
        }

        statusElement.textContent = 'Đang kết nối tới ROS...';
        const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
        ros.on('connection', () => {
            statusElement.textContent = 'Đã kết nối tới ROS';
            statusElement.style.backgroundColor = '#d4edda'; statusElement.style.color = '#155724';
        });
        ros.on('error', () => {
            statusElement.textContent = 'Lỗi kết nối ROS. Hãy chắc chắn rosbridge đang chạy.';
            statusElement.style.backgroundColor = '#f8d7da'; statusElement.style.color = '#721c24';
        });
        ros.on('close', () => {
            statusElement.textContent = 'Đã ngắt kết nối với ROS.';
            statusElement.style.backgroundColor = '#f8d7da'; statusElement.style.color = '#721c24';
        });

        const pathListener = new ROSLIB.Topic({ ros: ros, name: '/planned_path_nodes', messageType: 'std_msgs/String' });
        pathListener.subscribe((message) => {
            const nodes = message.data.split(',').map(node => node.trim()).filter(Boolean);
            if (nodes.length >= 2) {
                const totalPath = findTotalPath(nodes);
                drawPath(totalPath);
            } else {
                errorElement.textContent = "Dữ liệu nhận được không hợp lệ.";
                drawPath(null);
            }
        });
    }

    // Chạy hàm khởi tạo
    initialize();

}); // Dấu ngoặc đóng của 'DOMContentLoaded'