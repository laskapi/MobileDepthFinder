package overhorizon.mobiledepthfinder;

public final class  MultiValue <A,B,C,D>{
	
		private A first=null;
		private B second=null;
		private C third=null;
		private D fourth=null;
		private Boolean result=true;
		
		public MultiValue(Boolean result) {
			this.result=result;
		}
		
		public MultiValue(Boolean result,A first, B second) {
			this(result);
			this.first = first;
			this.second = second;
			
		
		}
		public MultiValue(Boolean result,A first, B second,C third) {
			this(result,first,second);
			this.third = third;
		
		}
		public MultiValue(Boolean result,A first, B second,C third,D fourth) {
			this(result,first,second,third);
			this.fourth = fourth;
		
		}
		
	
		public A getFirst() {
			return first;
		}
		
		public B getSecond() {
			return second;
		}
		public C getThird() {
			return third;
		}
		
		public D getFourth() {
			return fourth;
		}
		void setTrue(Boolean isTrue){
			this.result = isTrue;
		}
		boolean isTrue(){
			return this.result;
		}
		
}
