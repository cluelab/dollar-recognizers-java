# dollar-recognizers-java
Quick Java port of the $Q, $P and $P+ gesture recognizers. Based on the C# version from http://depts.washington.edu/madlab/proj/dollar/qdollar.html.

The academic publications for the recognizers are:
- Vatavu, R.-D., Anthony, L. and Wobbrock, J.O. (2012). Gestures as point clouds: A $P recognizer for user interface prototypes. Proceedings of the ACM Int'l Conference on Multimodal Interfaces (ICMI '12). Santa Monica, California (October 22-26, 2012). New York: ACM Press, pp. 273-280.
- Vatavu, R.-D. (2017). Improving Gesture Recognition Accuracy on Touch Screens for Users with Low Vision. In Proceedings of CHI '17, the 35th ACM Conference on Human Factors in Computing Systems (Denver, Colorado, USA, May 2017). New York: ACM Press. http://dx.doi.org/10.1145/3025453.3025941
- Vatavu, R.-D., Anthony, L. and Wobbrock, J.O. (2018). $Q: A Super-Quick, Articulation-Invariant Stroke-Gesture Recognizer for Low-Resource Devices. Proceedings of 20th International Conference on Human-Computer Interaction with Mobile Devices and Services (MobileHCI '18). Barcelona, Spain (September 3-6, 2018). New York: ACM Press. DOI: https://doi.org/10.1145/3229434.3229465

## Setup
You can use snapshot builds through [JitPack](https://jitpack.io):
* Go to [JitPack project page](https://jitpack.io/#cluelab/dollar-recognizers-java)
* Click `Get it` on commit you want to use (the top one is the most recent)
* Follow displayed instruction: add repository and dependency

#### Example (Gradle):
1. Add the JitPack repository to your root build.gradle at the end of repositories:
```gradle
	allprojects {
		repositories {
			...
			maven { url 'https://jitpack.io' }
		}
	}
```
2. Add the dependency
```gradle
	dependencies {
		implementation 'com.github.cluelab:dollar-recognizers-java:0b48f62a15'
	}
```

## Usage
The latest Javadoc is available at https://jitpack.io/com/github/cluelab/dollar-recognizers-java/-SNAPSHOT/javadoc/

#### Example
```java
import com.github.cluelab.dollar.*;
...
// define the training set or have it read from data files
Gesture[] trainingSet = new Gesture[] {
		new Gesture(new Point[] { /* point data here */ }, "arrow"),
		new Gesture(new Point[] { /* point data here */ }, "star"),
		new Gesture(new Point[] { /* point data here */ }, "asterisk"),
		/* more training samples here */
};

// acquire gesture points from user and construct the candidate gesture
Point[] points = new Point[] {
		/* points come from the acquisition device, e.g., mouse/pen/touch */
};
Gesture candidate = new Gesture(points);

// classify the candidate gesture with the preferred recognizer
// $P
String gestureClass = PointCloudRecognizer.Classify(candidate, trainingSet);
// $P+
String gestureClass = PointCloudRecognizerPlus.Classify(candidate, trainingSet);
// $Q
String gestureClass = QPointCloudRecognizer.Classify(candidate, trainingSet);
```

