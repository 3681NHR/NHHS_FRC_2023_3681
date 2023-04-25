package team3681.robot.lib.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.Callable;

/**
 * if this code is accidently left in, ignore it please.
 * Made it to show an example of an anagram sorter to a friend.
 * @deprecated
 * @hidden
 */
public class ParallelAnagramSorter {
    private static final int NUM_THREADS = Runtime.getRuntime().availableProcessors();

    public static List<String> generateAnagrams(String word) throws InterruptedException {
        List<String> anagrams = new ArrayList<>();
        ExecutorService executor = Executors.newFixedThreadPool(NUM_THREADS);
        generateAnagramsHelper("", word, anagrams, executor);
        executor.shutdown();
        executor.awaitTermination(1, TimeUnit.MINUTES);
        return anagrams;
    }

    private static void generateAnagramsHelper(String prefix, String suffix, List<String> anagrams, ExecutorService executor) {
        if (suffix.length() == 0) {
            anagrams.add(prefix);
        } else if (suffix.length() <= NUM_THREADS) {
            // If the remaining suffix is small enough, generate anagrams for it sequentially
            List<String> subAnagrams = new ArrayList<>();
            generateAnagramsHelper(prefix, suffix, subAnagrams, executor);
            anagrams.addAll(subAnagrams);
        } else {
            // Split the remaining suffix into multiple substrings and generate anagrams for each substring in parallel
            List<String> subSuffixes = splitSuffix(suffix, NUM_THREADS);
            List<Callable<Void>> tasks = new ArrayList<>();
            for (String subSuffix : subSuffixes) {
                Callable<Void> callable = Executors.callable(() -> generateAnagramsHelper(prefix, subSuffix, anagrams, executor), null);
                tasks.add(callable);
            }
            try {
                executor.invokeAll(tasks);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new RuntimeException("Anagram generation interrupted", e);
            }
        }
    }
    

    private static List<String> splitSuffix(String suffix, int numSubstrings) {
        List<String> subSuffixes = new ArrayList<>();
        int suffixLength = suffix.length();
        int subSuffixLength = (int) Math.ceil((double) suffixLength / numSubstrings);
        for (int i = 0; i < suffixLength; i += subSuffixLength) {
            int endIndex = Math.min(i + subSuffixLength, suffixLength);
            subSuffixes.add(suffix.substring(i, endIndex));
        }
        return subSuffixes;
    }
}
