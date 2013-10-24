#include "rendering_thread_manager.h"

#define A_THRESHOLD 0.01

RenderManager::RenderManager(int w, int h, int n)
: width(w), height(h), maxDepth(10), threshold(Vec3(0.01)), numOfThreads(n) {
    camera = new Camera(Vec3(8, 5, 8), Vec3(0, 0, 0), Vec3(0, 1, 0));
    camera->setSize(w, h);
    
    frontBuffer = new QImage(w, h, QImage::Format_RGB32);
    backBuffer = new QImage(w, h, QImage::Format_RGB32);
    
    rendering = false;
    
    colorBuffer = NULL;
}

RenderManager::~RenderManager() {
    delete [] tracers;
}

void RenderManager::addTask(RenderNode *n) {
    taskQueueMutex.lock();
    tasks.push(n);
    taskQueueMutex.unlock();
}

RenderNode* RenderManager::getTask() {
    taskQueueMutex.lock();
    RenderNode* task;
    if (tasks.size() > 0) {
         task = tasks.front();
        tasks.pop();
    } else {
        task = NULL;
    }
    taskQueueMutex.unlock();
    return task;
}

bool RenderManager::noTask() {
    taskQueueMutex.lock();
    bool empty = tasks.empty();
    taskQueueMutex.unlock();
    return empty;
}

void RenderManager::setPixelData(int x, int y, const Vec3& color) {
    PixelData& data = colorBuffer[x][y];
    if (data.rendered) {
        data.color += color;
    } else {
        data.color = color;
        data.rendered = true;
    }
    data.color.clamp();
    
    if (numOfRenderedNodes++ % 100000 == 0) {
        draw();
    }
}

void RenderManager::stopRendering() {
    rendering = false;
    for (int i = 0; i < numOfThreads; i++) {
        renderingThreads[i]->terminate();
    }
}

void RenderManager::clearTasks() {
    taskQueueMutex.lock();
    while (tasks.size() > 0) {
        tasks.pop();
    }
    taskQueueMutex.unlock();
}

void RenderManager::render() {
    //current_utc_time(&start);
    
    rendering = true;
    numOfRenderedNodes = 0;
    completedTracerCount = 0;
    
    while (!tasks.empty()) {
        tasks.pop();
    }
    
    refreshColorBuffer();
    
    /* initialize render nodes for the primary rays from the camera */
    /*
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            Ray* cameraRay = camera->getCameraRay(i, j);
            RenderNode* node = new RenderNode(cameraRay, i, j, 0, Vec3(1.f));
            addTask(node);
        }
    }
    */
    
    int centerX = width / 2;
    int centerY = height / 2;
    int distance = 0;
    
    while (true) {
        bool nodeAdded = false;
        
        for (int i = centerX - distance; i <= centerX + distance; i++) {
            for (int j = centerY - distance; j <= centerY + distance; j++) {
                if (i == centerX - distance || i == centerX + distance
                    || j == centerY - distance || j == centerY + distance) {
                    if (i >= 0 && i < width && j >= 0 && j < height) {
                        Ray* cameraRay = camera->getCameraRay(i, j);
                        RenderNode* node = new RenderNode(cameraRay, i, j, 0, Vec3(1));
                        addTask(node);
                        nodeAdded = true;
                    }
                }
            }
        }
        
        if (!nodeAdded) {
            break;
        }
        
        distance++;
    }
	startThreads();
}

void RenderManager::startThreads()
{
	    
    /* initialize all the tracers and threads */
    tracers = new RayTracer*[numOfThreads];
    renderingThreads = new QThread*[numOfThreads];
    
    /* schedule task for all the threads */
    for (int i = 0; i < numOfThreads; i++) {
        tracers[i] = new RayTracer(this, i);
        renderingThreads[i] = new QThread;
        tracers[i]->moveToThread(renderingThreads[i]);
        
        connect(renderingThreads[i], SIGNAL(started()), tracers[i], SLOT(run()));
        connect(tracers[i], SIGNAL(rayTraced(int, int, Vec3)), this, SLOT(tracerRayCompleted(int, int, Vec3)));
        connect(tracers[i], SIGNAL(completed()), this, SLOT(tracerCompleted()));
        
        renderingThreads[i]->start();
    }
}

void RenderManager::tracerRayCompleted(int x, int y, Vec3 c) {
    QColor origColor = backBuffer->pixel(x, y);

    backBuffer->setPixel(x, y, qRgb(c.x * 255 + origColor.red(),
        c.y * 255 + origColor.green(), c.z * 255 + origColor.blue()));
}

void RenderManager::tracerCompleted() {
    if (++completedTracerCount == numOfThreads) {
        // rendering = false;
        draw();
    }
	AA();
        
    // current_utc_time(&end);
    /*
     printf("s:  %lu\n", end.tv_sec - start.tv_sec);
     printf("ns: %lu\n", end.tv_sec - start.tv_nsec);
     */
    // cout << "time elapsed: " << float(clock() - startTime) / CLOCKS_PER_SEC << endl;
}

void RenderManager::draw() {
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            backBuffer->setPixel(j, i, qRgb(colorBuffer[j][i].color.x * 255,
                colorBuffer[j][i].color.y * 255, colorBuffer[j][i].color.z * 255));
        }
    }
    
    QImage* tmp = backBuffer;
    backBuffer = frontBuffer;
    frontBuffer = tmp;
    
    emit updateScreen();
}

void RenderManager::refreshColorBuffer() {
    if (colorBuffer == NULL) {
        colorBuffer = new PixelData*[height];
        for (int i = 0; i < height; i++) {
            colorBuffer[i] = new PixelData[width];
            for (int j = 0; j < width; j++) {
                colorBuffer[i][j] = PixelData(Vec3(0.f));
            }
        }
    }
    
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            // colorBuffer[i][j].color(Vec3(0.f));
            colorBuffer[i][j].rendered = false;
        }
    }
}

void RenderManager::current_utc_time(struct timespec *ts) {
    /*clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    ts->tv_sec = mts.tv_sec;
    ts->tv_nsec = mts.tv_nsec;*/
}


bool RenderManager::isAliasing(int i, int j)
{
	if (i > 0 && (colorBuffer[i - 1][j].color - colorBuffer[i][j].color).length() > A_THRESHOLD)
		return true;
	if (j > 0 && (colorBuffer[i][j - i].color - colorBuffer[i][j].color).length() > A_THRESHOLD)
		return true;
	if (i < width - 1 && (colorBuffer[i + 1][j].color - colorBuffer[i][j].color).length() > A_THRESHOLD)
		return true;
	if (j < height - 1 && (colorBuffer[i][j + 1].color - colorBuffer[i][j].color).length() > A_THRESHOLD)
		return true;
	return false;
}

void RenderManager::AA()
{
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (isAliasing(j, i))
			{
				colorBuffer[i][j].color = Vec3(0.0);
				Ray* r1 = camera->getCameraRay(i + 0.5, j);
				RenderNode* n1 = new RenderNode(r1, i, j, 0, Vec3(0.25f));
				tasks.push(n1);				
				Ray* r2 = camera->getCameraRay(i - 0.5, j);
				RenderNode* n2 = new RenderNode(r2, i, j, 0, Vec3(0.25f));
				tasks.push(n2);
				Ray* r3 = camera->getCameraRay(i, j + 0.5);
				RenderNode* n3 = new RenderNode(r3, i, j, 0, Vec3(0.25f));
				tasks.push(n3);				
				Ray* r4 = camera->getCameraRay(i, j - 0.5);
				RenderNode* n4 = new RenderNode(r4, i, j, 0, Vec3(0.25f));
				tasks.push(n4);
			}
		}
	}
	
	startThreads();
}