const mapImageElement = document.getElementById('mapImage');
const placeholderElement = document.getElementById('placeholder');
const updatedAtElement = document.getElementById('updatedAt');

let lastObjectUrl = null;

function showPlaceholder(message) {
  placeholderElement.textContent = message;
  placeholderElement.hidden = false;
  mapImageElement.hidden = true;
}

async function refreshMap() {
  try {
    const response = await fetch(`/api/map/latest?t=${Date.now()}`, {
      cache: 'no-store',
    });

    if (response.status === 404) {
      showPlaceholder('暂时还没有地图，请等待下一次保存。');
      updatedAtElement.textContent = '更新时间：-';
      return;
    }

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    const blob = await response.blob();
    const objectUrl = URL.createObjectURL(blob);
    if (lastObjectUrl) {
      URL.revokeObjectURL(lastObjectUrl);
    }
    lastObjectUrl = objectUrl;

    mapImageElement.src = objectUrl;
    mapImageElement.hidden = false;
    placeholderElement.hidden = true;
    updatedAtElement.textContent = `更新时间：${response.headers.get('X-Map-Updated-At') || '-'}`;
  } catch (error) {
    if (!lastObjectUrl) {
      showPlaceholder('地图加载失败，等待服务下一轮刷新。');
      updatedAtElement.textContent = '更新时间：-';
    }
  }
}

refreshMap();
setInterval(refreshMap, 5000);
